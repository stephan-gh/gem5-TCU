/*
 * Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
 * Copyright (C) 2019-2022 Nils Asmussen, Barkhausen Institut
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 */

#include "sim/m3_loader.hh"
#include "base/trace.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "debug/TcuTlb.hh"
#include "mem/port_proxy.hh"
#include "mem/tcu/tlb.hh"
#include "mem/tcu/tcu.hh"
#include "sim/byteswap.hh"
#include "sim/m3_system.hh"

#include <libgen.h>
#include <sstream>

namespace gem5
{

M3Loader::M3Loader(const std::vector<Addr> &tile_descs,
                   const std::vector<Addr> &tile_ids,
                   const std::vector<std::string> &mods,
                   const std::vector<std::string> &rot_layers,
                   const std::string &cmdline,
                   const std::string &logflags,
                   const std::string &kernel_cmdline,
                   Addr envStart,
                   tcu::TileId tileId,
                   Addr modOffset,
                   Addr modSize,
                   Addr tileSize)
    : tiles(),
      mods(mods),
      rotLayers(rot_layers),
      commandLine(cmdline),
      logflags(logflags),
      kernelCmdline(kernel_cmdline),
      envStart(envStart),
      tileId(tileId),
      modOffset(modOffset),
      modSize(modSize),
      tileSize(tileSize)
{
    assert(tile_descs.size() == tile_ids.size());
    for (size_t i = 0; i < tile_ids.size(); ++i)
        tiles[tcu::TileId::from_raw(tile_ids[i])] = tile_descs[i];
}

void
M3Loader::writeRemote(RequestPort &noc, Addr dest,
                      const uint8_t *data, size_t size)
{
    auto req = std::make_shared<Request>(dest, size, 0, Request::funcRequestorId);
    Packet pkt(req, MemCmd::WriteReq);
    pkt.dataStaticConst(data);

    auto senderState = new tcu::Tcu::NocSenderState();
    senderState->packetType = tcu::Tcu::NocPacketType::CACHE_MEM_REQ_FUNC;
    senderState->result = tcu::TcuError::NONE;

    pkt.pushSenderState(senderState);

    noc.sendFunctional(&pkt);

    delete senderState;
}

Addr
M3Loader::loadModule(RequestPort &noc, const std::string &filename, Addr addr)
{
    FILE *f = fopen(filename.c_str(), "r");
    if(!f)
        panic("Unable to open '%s' for reading", filename.c_str());

    fseek(f, 0L, SEEK_END);
    size_t sz = ftell(f);
    fseek(f, 0L, SEEK_SET);

    auto data = new uint8_t[sz];
    if(fread(data, 1, sz, f) != sz)
        panic("Unable to read '%s'", filename.c_str());
    writeRemote(noc, addr, data, sz);
    delete[] data;
    fclose(f);

    return sz;
}

void
M3Loader::writeArg(System &sys, Addr buf, size_t i, Addr argv,
                   const std::string &arg)
{
    // write argument pointer
    uint64_t argvPtr = buf;
    sys.physProxy.writeBlob(argv + i * sizeof(uint64_t),
                            (uint8_t*)&argvPtr, sizeof(argvPtr));
    if (buf)
    {
        // write argument
        sys.physProxy.writeBlob(buf, (uint8_t*)arg.c_str(), arg.length() + 1);
    }
}

Addr
M3Loader::writeArgs(System &sys, const std::vector<std::string> &args,
                    Addr argv, Addr bufStart, Addr bufEnd)
{
    size_t i = 0;
    Addr bufCur = bufStart;
    for (auto arg : args)
    {
        if(bufCur + arg.length() + 1 > bufEnd)
            panic("Not enough space for argv or envp.\n");
        writeArg(sys, bufCur, i, argv, arg);
        bufCur += arg.length() + 1;
        i++;
    }

    // null termination
    writeArg(sys, 0, i, argv, "");
    return bufCur;
}

Addr
M3Loader::loadModules(TileMemory &mem, RequestPort &noc, Addr offset,
                      BootModule *bmods)
{
    size_t i = 0;
    Addr addr = tcu::NocAddr(mem.memTile, offset).getAddr();
    for (const std::string &mod : mods)
    {
        // default --mods parameter leads to empty mod string
        if (mod.empty())
            continue;

        // split into name and path by "="
        std::stringstream ss(mod);
        std::string name, path;
        panic_if(!std::getline(ss, name, '='),
            "Unable to find '=' in module description");
        panic_if(!std::getline(ss, path),
            "Unable to read path from module description");

        Addr size = loadModule(noc, path, addr);

        // construct module info
        bmods[i].addr = addr;
        bmods[i].size = size;
        panic_if(name.length() >= MAX_MODNAME_LEN, "name too long");
        strcpy(bmods[i].name, name.c_str());

        inform("Loaded '%s' to %p .. %p",
            path, bmods[i].addr, bmods[i].addr + bmods[i].size);

        // to next
        addr += size + tcu::TcuTlb::PAGE_SIZE - 1;
        addr &= ~static_cast<Addr>(tcu::TcuTlb::PAGE_SIZE - 1);
        i++;
    }
    return addr;
}

void
M3Loader::initState(System &sys, TileMemory &mem, RequestPort &noc)
{
    // first initialize EP0 to a memory EP for our tile-local memory region in DRAM
    if (mem.initEpsAddr)
    {
        tcu::Ep ep;
        ep.mem.id = 0;
        ep.mem.r0.type = static_cast<tcu::RegFile::reg_t>(tcu::EpType::MEMORY);
        ep.mem.r0.act = tcu::Tcu::INVALID_ACT_ID;
        ep.mem.r0.flags = tcu::Tcu::MemoryFlags::READ | tcu::Tcu::MemoryFlags::WRITE;
        ep.mem.r0.targetTile = mem.memTile.raw();
        ep.mem.r1.remoteAddr = mem.memOffset;
        ep.mem.r2.remoteSize = mem.memSize;
        tcu::RegFile::printEpAccess(sys, ep, false, tcu::RegAccess::CPU);
        sys.physProxy.writeBlob(mem.initEpsAddr,
                    reinterpret_cast<const uint8_t*>(ep.inval.r), sizeof(ep.inval.r));
    }

    // init boot environment for the kernel
    BootEnv env;
    memset(&env, 0, sizeof(env));
    env.platform = Platform::GEM5;
    env.tile_id = tileId.raw();
    env.tile_desc = tile_attr(tileId);
    env.raw_tile_count = tiles.size();
    // write tile ids to environment
    {
        size_t i = 0;
        for(auto it = tiles.cbegin(); it != tiles.cend(); ++it, ++i)
            env.raw_tile_ids[i] = it->first.raw();
    }

    // convert command line to an array of strings
    std::istringstream iss(commandLine);
    std::vector<std::string> args{std::istream_iterator<std::string>(iss),
                                  std::istream_iterator<std::string>()};
    // build environment (only containing LOG=<logflags>)
    std::vector<std::string> envs;
    envs.push_back(std::string("LOG=") + logflags);

    Addr argv = envStart + sizeof(env);
    // the kernel gets the kernel env behind the normal env
    if (modSize)
        argv += sizeof(KernelEnv);

    // calculate argc, argv, and envp
    env.argc = args.size();
    env.argv = argv;
    env.envp = argv + sizeof(uint64_t) * (env.argc + 1);
    Addr bufStart = env.envp + sizeof(uint64_t) * 2;

    // write arguments and argument pointer to memory
    bufStart = writeArgs(sys, args, argv, bufStart, envStart + ENV_SIZE);
    writeArgs(sys, envs, env.envp, bufStart, envStart + ENV_SIZE);

    if (!rotLayers.empty())
    {
        static_assert(sizeof(RotCfg) <= tcu::TcuTlb::PAGE_SIZE,
                      "RoT configuration too large");
        auto offset = modOffset + tcu::TcuTlb::PAGE_SIZE; // For RotCfg
        RotBinary rotBins[2];
        panic_if(rotLayers.size() != 2, "Unexpected number of RoT layers");

        size_t i = 0;
        for (const std::string &layer : rotLayers)
        {
            Addr addr = tcu::NocAddr(mem.memTile, offset).getAddr();
            Addr size = loadModule(noc, layer, addr);
            rotBins[i].flash_offset = offset;
            rotBins[i].size = size;

            inform("Loaded RoT layer '%s' to %p .. %p",
                   layer, addr, addr + size);

            // to next
            offset += size + tcu::TcuTlb::PAGE_SIZE - 1;
            offset &= ~static_cast<Addr>(tcu::TcuTlb::PAGE_SIZE - 1);
            i++;
        }

        /*
         * Hard-coding the cached public keys/signatures and MACs here is a
         * workaround since there is no driver for persistently writing to the
         * flash in M3 yet. The values change whenever the hash of an involved
         * binary changes, which happens easily at the moment because M3 builds
         * are not reproducible (i.e. the binaries have different hashes when
         * built on different machines). Normally the cache fields should be
         * all zeroes and later filled on the first startup.
         *
         * To test caching, you need to manually fill the fields at the moment.
         * Note that having wrong values specified is no problem, the cache
         * will simply fail to validate and then the M3RoT layers regenerate
         * the signatures and print the new MACs to the console.
         *
         * To fill the values properly for a faster startup:
         *
         *  - Obtain the first three MACs printed to the console. In print
         *    order they represent the value of BLAU cache_mac, next_cache_mac
         *    and ROSA next_cache_mac, respectively. e.g.
         *    [C0T07:blau    @      56882] MAC: 87d2e792c4ced352ba003790c3ddf2712ab6a8930f3c17b7e4604bacd361ae83ac62753dafc702a67a995954db7d8dbcec8cf8abb78ff68180f149279d0ff51c
         *    [C0T07:blau    @      57767] MAC: 087a6f32cbbc6c2d6a08873567fee69a514864e35e0a9cfd10b78853848ac01969454039308d50b45c77e87fa7248669335f76de1624bbefaefb89f2b0c96df6
         *    [C0T07:rosa    @     153858] MAC: 394fa78e1e621026aa848d089950c6cf010c7159ed33f035ec09e6b32c6ad432642c07b54856719a286ae0cb5372a189f6fdefc92960452597cb42f9effcebf3
         *
         *  - Obtain public keys and signatures from the end of the first JSON
         *    payload that is printed to the console. e.g.
         *        // ROSA next_cache_pub_key
         *        "pub_key": "8046cb8d8d51ba796956031453cbba1613655ceee7ff42f205faa2917ed0da63"
         *      },
         *      // ROSA next_cache_signature
         *      "signature": "c2b33088c67d3c52b17f107cb9d47dd48830965e6e578ef8e760b2e8216f46d6fc279b905d4fb35beac375980f818967bfc4d81a388ffac275d2b54d1593560d",
         *      "pub_key": "2bcd48205bbea88f29093735d0cef5e8e983510181548f2f664b6b2af2cd8275",
         *      "parent": {
         *        "payload": {
         *          "type": "binary",
         *          "hash": "ff23738f410e7bfa1bd60cf8a5fe451af8f39817718d0cc91e4efcab9d02b89c",
         *          // BLAU next_cache_pub_key
         *          "pub_key": "2bcd48205bbea88f29093735d0cef5e8e983510181548f2f664b6b2af2cd8275"
         *        },
         *        // BLAU next_cache_signature
         *        "signature": "16c8ef0c3cff570feaa8e44489a1a486f8a67c3504a41b4f566a3611ff66d2f0e19f92c8096e655fe687834a57cc0e8ee5bc1826ff1fbfda10fc9605899f8402",
         *        // BLAU cache_pub_key
         *        "pub_key": "9f22b4921a5f7e5a53d3dea0aa254aded4da3ec7468a151c6ad3996298dfe040",
         *        "parent": null
         *      }
         *    }
         *
         * The hashes must be properly formatted as C++ array below. You can
         * try replacing ([0-9a-f]{2}) with "0x\1, " as a slight help.
         *
         * On a successfully cached run, no "MAC:" lines are printed to the log
         * and the system starts much faster.
         */

        constexpr size_t TCU_EPS_PER_PAGE = tcu::TcuTlb::PAGE_SIZE /
            (tcu::numEpRegs * sizeof(tcu::RegFile::reg_t));
        RotCfg cfg = {
            .brom = {
                .magic = 0x42726f6d43666701,
                .next_layer = rotBins[0],
            },
            .blau = {
                .magic = 0x426c617543666703,
                .next_layer = rotBins[1],
                .cache_pub_key = {0x9f, 0x22, 0xb4, 0x92, 0x1a, 0x5f, 0x7e, 0x5a, 0x53, 0xd3, 0xde, 0xa0, 0xaa, 0x25, 0x4a, 0xde, 0xd4, 0xda, 0x3e, 0xc7, 0x46, 0x8a, 0x15, 0x1c, 0x6a, 0xd3, 0x99, 0x62, 0x98, 0xdf, 0xe0, 0x40, },
                .cache_mac = {0x87, 0xd2, 0xe7, 0x92, 0xc4, 0xce, 0xd3, 0x52, 0xba, 0x00, 0x37, 0x90, 0xc3, 0xdd, 0xf2, 0x71, 0x2a, 0xb6, 0xa8, 0x93, 0x0f, 0x3c, 0x17, 0xb7, 0xe4, 0x60, 0x4b, 0xac, 0xd3, 0x61, 0xae, 0x83, 0xac, 0x62, 0x75, 0x3d, 0xaf, 0xc7, 0x02, 0xa6, 0x7a, 0x99, 0x59, 0x54, 0xdb, 0x7d, 0x8d, 0xbc, 0xec, 0x8c, 0xf8, 0xab, 0xb7, 0x8f, 0xf6, 0x81, 0x80, 0xf1, 0x49, 0x27, 0x9d, 0x0f, 0xf5, 0x1c, },
                .next_cache_pub_key = {0x2b, 0xcd, 0x48, 0x20, 0x5b, 0xbe, 0xa8, 0x8f, 0x29, 0x09, 0x37, 0x35, 0xd0, 0xce, 0xf5, 0xe8, 0xe9, 0x83, 0x51, 0x01, 0x81, 0x54, 0x8f, 0x2f, 0x66, 0x4b, 0x6b, 0x2a, 0xf2, 0xcd, 0x82, 0x75, },
                .next_cache_signature = {0x16, 0xc8, 0xef, 0x0c, 0x3c, 0xff, 0x57, 0x0f, 0xea, 0xa8, 0xe4, 0x44, 0x89, 0xa1, 0xa4, 0x86, 0xf8, 0xa6, 0x7c, 0x35, 0x04, 0xa4, 0x1b, 0x4f, 0x56, 0x6a, 0x36, 0x11, 0xff, 0x66, 0xd2, 0xf0, 0xe1, 0x9f, 0x92, 0xc8, 0x09, 0x6e, 0x65, 0x5f, 0xe6, 0x87, 0x83, 0x4a, 0x57, 0xcc, 0x0e, 0x8e, 0xe5, 0xbc, 0x18, 0x26, 0xff, 0x1f, 0xbf, 0xda, 0x10, 0xfc, 0x96, 0x05, 0x89, 0x9f, 0x84, 0x02, },
                .next_cache_mac = {0x08, 0x7a, 0x6f, 0x32, 0xcb, 0xbc, 0x6c, 0x2d, 0x6a, 0x08, 0x87, 0x35, 0x67, 0xfe, 0xe6, 0x9a, 0x51, 0x48, 0x64, 0xe3, 0x5e, 0x0a, 0x9c, 0xfd, 0x10, 0xb7, 0x88, 0x53, 0x84, 0x8a, 0xc0, 0x19, 0x69, 0x45, 0x40, 0x39, 0x30, 0x8d, 0x50, 0xb4, 0x5c, 0x77, 0xe8, 0x7f, 0xa7, 0x24, 0x86, 0x69, 0x33, 0x5f, 0x76, 0xde, 0x16, 0x24, 0xbb, 0xef, 0xae, 0xfb, 0x89, 0xf2, 0xb0, 0xc9, 0x6d, 0xf6, },
            },
            .rosa = {
                .magic = 0x526f736143666703,
                .next_cache_pub_key = {0x80, 0x46, 0xcb, 0x8d, 0x8d, 0x51, 0xba, 0x79, 0x69, 0x56, 0x03, 0x14, 0x53, 0xcb, 0xba, 0x16, 0x13, 0x65, 0x5c, 0xee, 0xe7, 0xff, 0x42, 0xf2, 0x05, 0xfa, 0xa2, 0x91, 0x7e, 0xd0, 0xda, 0x63, },
                .next_cache_signature = {0xc2, 0xb3, 0x30, 0x88, 0xc6, 0x7d, 0x3c, 0x52, 0xb1, 0x7f, 0x10, 0x7c, 0xb9, 0xd4, 0x7d, 0xd4, 0x88, 0x30, 0x96, 0x5e, 0x6e, 0x57, 0x8e, 0xf8, 0xe7, 0x60, 0xb2, 0xe8, 0x21, 0x6f, 0x46, 0xd6, 0xfc, 0x27, 0x9b, 0x90, 0x5d, 0x4f, 0xb3, 0x5b, 0xea, 0xc3, 0x75, 0x98, 0x0f, 0x81, 0x89, 0x67, 0xbf, 0xc4, 0xd8, 0x1a, 0x38, 0x8f, 0xfa, 0xc2, 0x75, 0xd2, 0xb5, 0x4d, 0x15, 0x93, 0x56, 0x0d, },
                .next_cache_mac = {0x39, 0x4f, 0xa7, 0x8e, 0x1e, 0x62, 0x10, 0x26, 0xaa, 0x84, 0x8d, 0x08, 0x99, 0x50, 0xc6, 0xcf, 0x01, 0x0c, 0x71, 0x59, 0xed, 0x33, 0xf0, 0x35, 0xec, 0x09, 0xe6, 0xb3, 0x2c, 0x6a, 0xd4, 0x32, 0x64, 0x2c, 0x07, 0xb5, 0x48, 0x56, 0x71, 0x9a, 0x28, 0x6a, 0xe0, 0xcb, 0x53, 0x72, 0xa1, 0x89, 0xf6, 0xfd, 0xef, 0xc9, 0x29, 0x60, 0x45, 0x25, 0x97, 0xcb, 0x42, 0xf9, 0xef, 0xfc, 0xeb, 0xf3, },
                .kernel_mem_size = tileSize,
                .kernel_ep_pages = static_cast<uint8_t>(
                    // For now just configure the kernel tile with the same
                    // amount of EPs as used for the RoT tile (could also make
                    // this separately configurable).
                    std::min(divCeil(mem.initEpsNum, TCU_EPS_PER_PAGE),
                             static_cast<size_t>(UINT8_MAX))),
                .kernel_cmdline = {},
                .mods = {},
            },
        };

        panic_if(mods.size() > (sizeof(cfg.rosa.mods) / sizeof(cfg.rosa.mods[0])),
                 "Too many modules for RoT");
        panic_if(kernelCmdline.length() >= sizeof(cfg.rosa.kernel_cmdline) - 1,
                 "Kernel command line too long");
        strcpy(cfg.rosa.kernel_cmdline, kernelCmdline.c_str());

        Addr addr = loadModules(mem, noc, offset, cfg.rosa.mods);
        Addr end = tcu::NocAddr(mem.memTile, modOffset + modSize).getAddr();
        panic_if(addr > end, "Modules are too large (have: %lu, need: %lu)",
                 modSize, addr - tcu::NocAddr(mem.memTile, modOffset).getAddr());

        writeRemote(noc, tcu::NocAddr(mem.memTile, modOffset).getAddr(),
                    reinterpret_cast<uint8_t*>(&cfg), sizeof(cfg));
    }
    // modules for the kernel
    else if (modSize)
    {
        BootModule *bmods = new BootModule[mods.size()]();
        Addr addr = loadModules(mem, noc, modOffset, bmods);

        // determine memory regions
        size_t mem_count = 0;
        MemMod *bmems = new MemMod[tiles.size()];
        auto avail_mem_start = modOffset + modSize + tileSize;
        bmems[0].size = (tile_attr(mem.memTile) >> 28) << 12;
        if (bmems[0].size < avail_mem_start)
            panic("Not enough DRAM for modules and tiles");
        bmems[0].addr = tcu::NocAddr(mem.memTile, avail_mem_start).getAddr();
        bmems[0].size -= avail_mem_start;
        mem_count++;

        for(auto it = tiles.cbegin(); it != tiles.cend(); ++it)
        {
            if (it->first != mem.memTile && (tile_attr(it->first) & 0x3F) == 1) {
                bmems[mem_count].addr = tcu::NocAddr(it->first, 0).getAddr();
                bmems[mem_count].size = (tile_attr(it->first) >> 28) << 12;
                mem_count++;
            }
        }

        // write kenv
        env.kenv = addr;
        KernelEnv kenv;
        kenv.mod_count = mods.size();
        kenv.tile_count  = tiles.size();
        kenv.mem_count = mem_count;
        kenv.serv_count = 0;
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(&kenv),
                    sizeof(kenv));
        addr += sizeof(kenv);

        // write modules to memory
        size_t bmodsize = kenv.mod_count * sizeof(BootModule);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(bmods), bmodsize);
        delete[] bmods;
        addr += bmodsize;

        // write tile descriptors to memory
        uint64_t *ktileDescs = new uint64_t[kenv.tile_count]();
        {
            size_t i = 0;
            for(auto it = tiles.cbegin(); it != tiles.cend(); ++it, ++i)
                ktileDescs[i] = it->second;
        }
        size_t bdescssize = kenv.tile_count * sizeof(uint64_t);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(ktileDescs), bdescssize);
        delete[] ktileDescs;
        addr += bdescssize;

        // write memory regions to memory
        size_t bmemsize = kenv.mem_count * sizeof(MemMod);
        writeRemote(noc, addr, reinterpret_cast<uint8_t*>(bmems), bmemsize);
        delete[] bmems;
        addr += bmemsize;

        // check size
        Addr end = tcu::NocAddr(mem.memTile, modOffset + modSize).getAddr();
        if (addr > end)
        {
            panic("Modules are too large (have: %lu, need: %lu)",
                modSize, addr - tcu::NocAddr(mem.memTile, modOffset).getAddr());
        }
    }

    // write env
    sys.physProxy.writeBlob(
        envStart, reinterpret_cast<uint8_t*>(&env), sizeof(env));
}

}
