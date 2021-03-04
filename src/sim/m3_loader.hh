/*
 * Copyright (c) 2015, Nils Asmussen
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

#ifndef __SIM_M3_LOADER_HH__
#define __SIM_M3_LOADER_HH__

#include <string>
#include <vector>

#include "sim/system.hh"
#include "mem/tcu/noc_addr.hh"
#include "sim/pe_memory.hh"

class M3Loader
{
  protected:
    static const size_t ENV_SIZE        = 0x1000;
#if THE_ISA == RISCV_ISA
    static const uintptr_t ENV_START    = 0x10100000;
#else
    static const uintptr_t ENV_START    = 0x100000;
#endif
    static const size_t HEAP_SIZE       = 0x8000;
    static const size_t MAX_MODNAME_LEN = 64;

    struct BootModule
    {
        uint64_t addr;
        uint64_t size;
        char name[MAX_MODNAME_LEN];
    } M5_ATTR_PACKED;

    struct MemMod
    {
        uint64_t addr;
        uint64_t size;
    } M5_ATTR_PACKED;

    struct KernelEnv
    {
        uint64_t mod_count;
        uint64_t pe_count;
        uint64_t mem_count;
        uint64_t serv_count;
    } M5_ATTR_PACKED;

    enum Platform
    {
        GEM5,
        HW
    };

    struct BootEnv
    {
        uint64_t platform;
        uint64_t pe_id;
        uint64_t pe_desc;
        uint64_t argc;
        uint64_t argv;
        uint64_t heap_size;
        uint64_t kenv;
        uint64_t lambda;
    } M5_ATTR_PACKED;

    std::vector<Addr> pes;
    std::vector<std::string> mods;
    std::string commandLine;

  public:
    const uint coreId;
    const Addr modOffset;
    const Addr modSize;
    const Addr peSize;

  public:
    M3Loader(const std::vector<Addr> &pes,
             const std::vector<std::string> &mods,
             const std::string &cmdline,
             uint coreId,
             Addr modOffset,
             Addr modSize,
             Addr peSize);

    const std::vector<Addr> &pe_attr() const
    {
        return pes;
    }

    void initState(System &sys, PEMemory &mem, MasterPort &noc);

  private:
    size_t getArgc() const;
    void writeArg(System &sys, Addr &args, size_t &i, Addr argv,
                  const char *cmd, const char *begin);
    void writeRemote(MasterPort &noc, Addr dest,
                     const uint8_t *data, size_t size);
    Addr loadModule(MasterPort &noc, const std::string &filename, Addr addr);
};

#endif
