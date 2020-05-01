/*
 * Copyright (c) 2015, Christian Menard
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

#include <iomanip>
#include <sstream>

#include "debug/Tcu.hh"
#include "debug/TcuBuf.hh"
#include "debug/TcuCmd.hh"
#include "debug/TcuPackets.hh"
#include "debug/TcuMem.hh"
#include "debug/TcuCoreMemAcc.hh"
#include "mem/tcu/tcu.hh"
#include "mem/tcu/msg_unit.hh"
#include "mem/tcu/mem_unit.hh"
#include "mem/tcu/xfer_unit.hh"
#include "mem/cache/cache.hh"
#include "sim/pe_memory.hh"
#include "sim/system.hh"

static const char *cmdNames[] =
{
    "IDLE",
    "SEND",
    "REPLY",
    "READ",
    "WRITE",
    "FETCH_MSG",
    "ACK_MSG",
    "SLEEP",
};

static const char *privCmdNames[] =
{
    "IDLE",
    "INV_PAGE",
    "INV_TLB",
    "INS_TLB",
    "XCHG_VPE",
    "FLUSH_CACHE",
    "SET_TIMER",
    "ABORT_CMD",
};

static const char *extCmdNames[] =
{
    "IDLE",
    "INV_EP",
    "INV_REPLY",
    "RESET",
};

Tcu::Tcu(TcuParams* p)
  : BaseTcu(p),
    masterId(p->system->getMasterId(this, name())),
    system(p->system),
    regFile(*this, name() + ".regFile", p->num_endpoints),
    connector(p->connector),
    tlBuf(p->tlb_entries > 0 ? new TcuTlb(*this, p->tlb_entries) : NULL),
    msgUnit(new MessageUnit(*this)),
    memUnit(new MemoryUnit(*this)),
    xferUnit(new XferUnit(*this, p->block_size, p->buf_count, p->buf_size)),
    coreReqs(*this, p->buf_count),
    fireTimerEvent(*this),
    completeCoreReqEvent(coreReqs),
    cmdPkt(),
    privCmdPkt(),
    cmdFinish(),
    abort(),
    cmdIsRemote(),
    wakeupEp(0xFFFF),
    memPe(),
    memOffset(),
    atomicMode(p->system->isAtomicMode()),
    numEndpoints(p->num_endpoints),
    maxNocPacketSize(p->max_noc_packet_size),
    blockSize(p->block_size),
    bufCount(p->buf_count),
    bufSize(p->buf_size),
    reqCount(p->req_count),
    cacheBlocksPerCycle(p->cache_blocks_per_cycle),
    registerAccessLatency(p->register_access_latency),
    cpuToCacheLatency(p->cpu_to_cache_latency),
    commandToNocRequestLatency(p->command_to_noc_request_latency),
    startMsgTransferDelay(p->start_msg_transfer_delay),
    transferToMemRequestLatency(p->transfer_to_mem_request_latency),
    transferToNocLatency(p->transfer_to_noc_latency),
    nocToTransferLatency(p->noc_to_transfer_latency)
{
    static_assert(sizeof(cmdNames) / sizeof(cmdNames[0]) ==
        CmdCommand::SLEEP + 1, "cmdNames out of sync");
    static_assert(sizeof(privCmdNames) / sizeof(privCmdNames[0]) ==
        PrivCommand::ABORT_CMD + 1, "privCmdNames out of sync");
    static_assert(sizeof(extCmdNames) / sizeof(extCmdNames[0]) ==
        ExtCommand::RESET + 1, "extCmdNames out of sync");

    assert(p->buf_size >= maxNocPacketSize);

    connector->setTcu(this);

    PEMemory *sys = dynamic_cast<PEMemory*>(system);
    if (sys)
    {
        NocAddr phys = sys->getPhys(0);
        memPe = phys.peId;
        memOffset = phys.offset;
        memSize = sys->memSize;
        DPRINTF(Tcu, "Using memory range %p .. %p\n",
            memOffset, memOffset + memSize);
    }
}

Tcu::~Tcu()
{
    delete xferUnit;
    delete memUnit;
    delete msgUnit;
    delete tlBuf;
}

void
Tcu::regStats()
{
    BaseTcu::regStats();

    nocMsgRecvs
        .name(name() + ".nocMsgRecvs")
        .desc("Number of received messages");
    nocReadRecvs
        .name(name() + ".nocReadRecvs")
        .desc("Number of received read requests");
    nocWriteRecvs
        .name(name() + ".nocWriteRecvs")
        .desc("Number of received write requests");

    regFileReqs
        .name(name() + ".regFileReqs")
        .desc("Number of requests to the register file");
    intMemReqs
        .name(name() + ".intMemReqs")
        .desc("Number of requests to the internal memory");
    extMemReqs
        .name(name() + ".extMemReqs")
        .desc("Number of requests to the external memory");
    irqInjects
        .name(name() + ".irqInjects")
        .desc("Number of injected IRQs");
    resets
        .name(name() + ".resets")
        .desc("Number of resets");

    commands
        .init(sizeof(cmdNames) / sizeof(cmdNames[0]))
        .name(name() + ".commands")
        .desc("The executed commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(cmdNames) / sizeof(cmdNames[0]); ++i)
        commands.subname(i, cmdNames[i]);

    privCommands
        .init(sizeof(privCmdNames) / sizeof(privCmdNames[0]))
        .name(name() + ".privCommands")
        .desc("The executed privileged commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(privCmdNames) / sizeof(privCmdNames[0]); ++i)
        privCommands.subname(i, privCmdNames[i]);

    extCommands
        .init(sizeof(extCmdNames) / sizeof(extCmdNames[0]))
        .name(name() + ".extCommands")
        .desc("The executed external commands")
        .flags(Stats::total | Stats::nozero);
    for (size_t i = 0; i < sizeof(extCmdNames) / sizeof(extCmdNames[0]); ++i)
        extCommands.subname(i, extCmdNames[i]);

    if (tlb())
        tlb()->regStats();
    coreReqs.regStats();
    xferUnit->regStats();
    memUnit->regStats();
    msgUnit->regStats();
}

bool
Tcu::isMemPE(unsigned pe) const
{
    PEMemory *sys = dynamic_cast<PEMemory*>(system);
    return !sys || sys->hasMem(pe);
}

PacketPtr
Tcu::generateRequest(Addr paddr, Addr size, MemCmd cmd)
{
    Request::Flags flags;

    auto req = std::make_shared<Request>(paddr, size, flags, masterId);

    auto pkt = new Packet(req, cmd);

    if (size)
    {
        auto pktData = new uint8_t[size];
        pkt->dataDynamic(pktData);
    }

    return pkt;
}

void
Tcu::freeRequest(PacketPtr pkt)
{
    delete pkt;
}

void
Tcu::executeCommand(PacketPtr pkt)
{
    CmdCommand::Bits cmd = regs().getCommand();
    if (cmd.opcode == CmdCommand::IDLE)
    {
        if (pkt)
            schedCpuResponse(pkt, clockEdge(Cycles(1)));
        return;
    }

    assert(cmdPkt == nullptr);
    cmdPkt = pkt;
    commands[static_cast<size_t>(cmd.opcode)]++;

    assert(cmd.epid < numEndpoints);
    DPRINTF(TcuCmd, "Starting command %s with EP=%u, arg=%#lx\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid, cmd.arg);

    switch (cmd.opcode)
    {
        case CmdCommand::SEND:
        case CmdCommand::REPLY:
            msgUnit->startTransmission(cmd);
            break;
        case CmdCommand::READ:
            memUnit->startRead(cmd);
            break;
        case CmdCommand::WRITE:
            memUnit->startWrite(cmd);
            break;
        case CmdCommand::FETCH_MSG:
            regs().set(CmdReg::ARG1, msgUnit->fetchMessage(cmd.epid));
            finishCommand(TcuError::NONE);
            break;
        case CmdCommand::ACK_MSG:
            finishCommand(msgUnit->ackMessage(cmd.epid, cmd.arg));
            break;
        case CmdCommand::SLEEP:
        {
            int ep = cmd.arg & 0xFFFF;
            if (!startSleep(ep))
                finishCommand(TcuError::NONE);
        }
        break;
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (cmdPkt && cmd.opcode != CmdCommand::SLEEP)
    {
        if (connector->canSuspendCmds())
            startSleep(INVALID_EP_ID);
        else
        {
            schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));
            cmdPkt = nullptr;
        }
    }
}

void
Tcu::abortCommand()
{
    CmdCommand::Bits cmd = regs().getCommand();

    // if we've already scheduled finishCommand, consider the command done
    if (!cmdFinish && cmd.opcode != CmdCommand::IDLE)
    {
        // if we are waiting for a NoC response, just wait until we receive it.
        // we deem this acceptable, because these remotely running transfers
        // never cause page faults and thus complete in short amounts of time.
        if (cmdIsRemote)
        {
            // SEND/REPLY needs to finish successfully as soon as we've sent
            // out the message.
            if (cmd.opcode != CmdCommand::SEND &&
                cmd.opcode != CmdCommand::REPLY)
                abort = AbortType::REMOTE;
        }
        // otherwise, abort it locally. this is done for all commands, because
        // all can cause page faults locally.
        else
        {
            abort = AbortType::LOCAL;

            auto res = xferUnit->tryAbortCommand();
            // if the current command used the xferUnit, we're done
            if (res == XferUnit::AbortResult::ABORTED)
                scheduleFinishOp(Cycles(1), TcuError::ABORT);
        }

        if (abort != AbortType::NONE)
        {
            DPRINTF(TcuCmd, "Aborting command %s with EP=%u\n",
                    cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid);
        }
    }
    else
    {
        abort = AbortType::NONE;
        // don't do that if we'll do that in finishCommand event anyway
        if (!cmdFinish)
            finishAbort();
    }
}

void
Tcu::scheduleFinishOp(Cycles delay, TcuError error)
{
    if (regs().getCommand().opcode != CmdCommand::IDLE)
    {
        if (cmdFinish)
        {
            deschedule(cmdFinish);
            delete cmdFinish;
        }

        if (delay == 0)
            finishCommand(error);
        else
        {
            cmdFinish = new FinishCommandEvent(*this, error);
            schedule(cmdFinish, clockEdge(delay));
        }
    }
}

void
Tcu::finishCommand(TcuError error)
{
    CmdCommand::Bits cmd = regs().getCommand();

    cmdFinish = NULL;

    if (cmd.opcode == CmdCommand::SEND)
        msgUnit->finishMsgSend(error, cmd.epid);
    else if (cmd.opcode == CmdCommand::REPLY)
        msgUnit->finishMsgReply(error, cmd.epid, cmd.arg);
    else if (error == TcuError::NONE &&
             (cmd.opcode == CmdCommand::READ || cmd.opcode == CmdCommand::WRITE))
    {
        const CmdData::Bits data = regs().getData();
        if (data.size > 0)
        {
            if (cmd.opcode == CmdCommand::READ)
                memUnit->startRead(cmd);
            else
                memUnit->startWrite(cmd);
            return;
        }
    }

    if (cmdPkt || cmd.opcode == CmdCommand::SLEEP)
        stopSleep();

    DPRINTF(TcuCmd, "Finished command %s with EP=%u -> %u\n",
            cmdNames[static_cast<size_t>(cmd.opcode)], cmd.epid,
            static_cast<uint>(error));

    // let the SW know that the command is finished
    cmd = 0;
    cmd.error = static_cast<unsigned>(error);
    cmd.opcode = CmdCommand::IDLE;
    regFile.set(CmdReg::COMMAND, cmd);

    if (cmdPkt)
        schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));
    cmdPkt = NULL;

    // finish command abortion, if there is any
    finishAbort();
}

void
Tcu::executePrivCommand(PacketPtr pkt)
{
    PrivCommand::Bits cmd = regFile.get(PrivReg::PRIV_CMD);

    privCommands[static_cast<size_t>(cmd.opcode)]++;

    DPRINTF(TcuCmd, "Executing privileged command %s with arg=%p\n",
            privCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg);

    Cycles delay(1);

    switch (cmd.opcode)
    {
        case PrivCommand::IDLE:
            break;
        case PrivCommand::INV_PAGE:
            if (tlb())
            {
                uint16_t asid = cmd.arg >> 44;
                Addr virt = cmd.arg & 0xFFFFFFFFFFF;
                tlb()->remove(virt, asid);
            }
            break;
        case PrivCommand::INV_TLB:
            if (tlb())
                tlb()->clear();
            break;
        case PrivCommand::INS_TLB:
            if (tlb())
            {
                uint16_t asid = cmd.arg >> 44;
                Addr virt = cmd.arg & 0xFFFFFFFF000;
                uint flags = cmd.arg & 0x1F;
                Addr phys = regFile.get(PrivReg::PRIV_CMD_ARG);
                tlb()->insert(virt, asid, NocAddr(phys), flags);
            }
            break;
        case PrivCommand::XCHG_VPE:
        {
            RegFile::reg_t old = regs().get(PrivReg::CUR_VPE);
            regs().set(PrivReg::OLD_VPE, old);
            regs().set(PrivReg::CUR_VPE, cmd.arg & 0xFFFFFFFF);
            break;
        }
        case PrivCommand::FLUSH_CACHE:
            delay += flushInvalCaches(true);
            break;
        case PrivCommand::SET_TIMER:
        {
            if (fireTimerEvent.scheduled())
                deschedule(&fireTimerEvent);
            if (cmd.arg != 0)
            {
                Cycles sleep_time = ticksToCycles(cmd.arg * 1000);
                schedule(&fireTimerEvent, clockEdge(sleep_time));
            }
            break;
        }
        case PrivCommand::ABORT_CMD:
        {
            privCmdPkt = pkt;
            pkt = nullptr;
            abortCommand();
            return;
        }
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
        schedCpuResponse(pkt, clockEdge(delay));

    // set privileged command back to IDLE
    cmd.arg = 0;
    cmd.opcode = PrivCommand::IDLE;
    regFile.set(PrivReg::PRIV_CMD, cmd);

    DPRINTF(TcuCmd, "Finished privileged command %s with res=0\n",
            privCmdNames[static_cast<size_t>(cmd.opcode)]);
}

void
Tcu::finishAbort()
{
    if (privCmdPkt != nullptr)
    {
        schedCpuResponse(privCmdPkt, clockEdge(Cycles(1)));
        privCmdPkt = nullptr;

        PrivCommand::Bits cmd = regFile.get(PrivReg::PRIV_CMD);
        cmd.arg = static_cast<RegFile::reg_t>(abort);

        DPRINTF(TcuCmd, "Finished privileged command %s with res=%d\n",
                privCmdNames[static_cast<size_t>(cmd.opcode)],
                cmd.arg);

        cmd.opcode = PrivCommand::IDLE;
        regFile.set(PrivReg::PRIV_CMD, cmd);

        abort = AbortType::NONE;
    }
}

void
Tcu::executeExtCommand(PacketPtr pkt)
{
    ExtCommand::Bits cmd = regFile.get(PrivReg::EXT_CMD);

    extCommands[static_cast<size_t>(cmd.opcode)]++;

    Cycles delay(1);

    DPRINTF(TcuCmd, "Executing external command %s with arg=%p\n",
            extCmdNames[static_cast<size_t>(cmd.opcode)], cmd.arg);

    switch (cmd.opcode)
    {
        case ExtCommand::IDLE:
            break;
        case ExtCommand::INV_EP:
        {
            epid_t epid = cmd.arg & 0xFFFF;
            bool force = !!(cmd.arg & (1 << 16));
            unsigned unreadMask;
            TcuError res = regs().invalidate(epid, force, &unreadMask);
            cmd.error = static_cast<uint>(res);
            cmd.arg = unreadMask;
            break;
        }
        case ExtCommand::INV_REPLY:
        {
            epid_t repid = cmd.arg & 0xFFFF;
            peid_t peid = (cmd.arg >> 16) & 0xFF;
            epid_t sepid = (cmd.arg >> 24) & 0xFFFF;
            TcuError res = msgUnit->invalidateReply(repid, peid, sepid);
            cmd.error = static_cast<uint>(res);
            break;
        }
        case ExtCommand::RESET:
            delay += reset(cmd.arg != 0);
            break;
        default:
            // TODO error handling
            panic("Invalid opcode %#x\n", static_cast<RegFile::reg_t>(cmd.opcode));
    }

    if (pkt)
    {
        auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);
        assert(senderState);
        schedNocResponse(pkt, clockEdge(delay));
    }

    if (cmd.error != static_cast<uint>(TcuError::NONE))
    {
        DPRINTF(TcuCmd, "External command %s failed (%u)\n",
                extCmdNames[static_cast<size_t>(cmd.opcode)],
                static_cast<uint>(cmd.error));
    }

    // set external command back to IDLE
    cmd.opcode = ExtCommand::IDLE;
    regFile.set(PrivReg::EXT_CMD, cmd);
}

bool
Tcu::has_message(epid_t ep)
{
    if (ep == 0xFFFF && regFile.messages() > 0)
        return true;

    if (ep != 0xFFFF)
    {
        RecvEp *rep = regFile.getRecvEp(ep);
        if (rep && rep->r2.unread != 0)
            return true;
    }
    return false;
}

bool
Tcu::startSleep(epid_t ep)
{
    if (has_message(ep))
        return false;
    if (connector->havePendingIrq())
        return false;

    wakeupEp = ep;
    DPRINTF(Tcu, "Suspending CU (waiting for EP %d)\n", wakeupEp);
    connector->suspend();

    return true;
}

void
Tcu::stopSleep()
{
    connector->wakeup();
}

void
Tcu::wakeupCore(bool force)
{
    if (force || has_message(wakeupEp))
    {
        // better stop the command in this cycle to ensure that the core
        // does not issue another command before we can finish the sleep.
        if (regs().getCommand().opcode == CmdCommand::SLEEP)
            scheduleFinishOp(Cycles(0));
        else
            connector->wakeup();
    }
}

Cycles
Tcu::reset(bool flushInval)
{
    Cycles delay = flushInval ? flushInvalCaches(true) : Cycles(0);

    if (tlb())
        tlb()->clear();

    connector->reset();

    resets++;
    return delay;
}

Cycles
Tcu::flushInvalCaches(bool invalidate)
{
    Cycles delay(0);
    for (auto &c : caches)
    {
        c->memWriteback();
        if (invalidate)
            c->memInvalidate();
        delay += Cycles(c->getBlockCount() / cacheBlocksPerCycle);
    }
    return delay;
}

void
Tcu::setIrq(BaseConnector::IRQ irq)
{
    wakeupCore(true);

    connector->setIrq(irq);

    irqInjects++;
}

void
Tcu::clearIrq(BaseConnector::IRQ irq)
{
    connector->clearIrq(irq);
}

void
Tcu::fireTimer()
{
    setIrq(BaseConnector::IRQ::TIMER);
}

void
Tcu::printLine(Addr len)
{
    const char *buffer = regs().getBuffer(len);
    DPRINTF(Tcu, "PRINT: %s\n", buffer);
}

void
Tcu::sendMemRequest(PacketPtr pkt,
                    Addr virt,
                    Addr data,
                    Cycles delay)
{
    auto senderState = new MemSenderState();
    senderState->data = data;
    senderState->mid = pkt->req->masterId();

    // ensure that this packet has our master id (not the id of a master in
    // a different PE)
    pkt->req->setMasterId(masterId);

    pkt->pushSenderState(senderState);

    if (atomicMode)
    {
        sendAtomicMemRequest(pkt);
        completeMemRequest(pkt);
    }
    else
    {
        schedMemRequest(pkt, clockEdge(delay));
    }
}

void
Tcu::sendNocRequest(NocPacketType type,
                    PacketPtr pkt,
                    vpeid_t tvpe,
                    Cycles delay,
                    bool functional)
{
    auto senderState = new NocSenderState();
    senderState->packetType = type;
    senderState->result = TcuError::NONE;
    senderState->tvpe = tvpe;

    if (type == NocPacketType::MESSAGE || type == NocPacketType::READ_REQ ||
        type == NocPacketType::WRITE_REQ)
        cmdIsRemote = true;

    pkt->pushSenderState(senderState);

    if (functional)
    {
        sendFunctionalNocRequest(pkt);
        completeNocRequest(pkt);
    }
    else if (atomicMode)
    {
        sendAtomicNocRequest(pkt);
        completeNocRequest(pkt);
    }
    else
    {
        schedNocRequest(pkt, clockEdge(delay));
    }
}

void
Tcu::sendNocResponse(PacketPtr pkt)
{
    pkt->makeResponse();

    if (!atomicMode)
    {
        Cycles delay = ticksToCycles(
            pkt->headerDelay + pkt->payloadDelay);
        delay += nocToTransferLatency;

        pkt->headerDelay = 0;
        pkt->payloadDelay = 0;

        schedNocRequestFinished(clockEdge(Cycles(1)));
        schedNocResponse(pkt, clockEdge(delay));
    }
}

Addr
Tcu::physToNoc(Addr phys)
{
#if THE_ISA == X86_ISA
    return (phys & ~0x0000FF0000000000ULL) |
          ((phys & 0x0000FF0000000000ULL) << 16);
#elif THE_ISA == ARM_ISA
    return (phys & ~0x000000FF00000000ULL) |
          ((phys & 0x000000FF00000000ULL) << 24);
#elif THE_ISA == RISCV_ISA
    return (phys & ~0x00FF000000000000ULL) |
          ((phys & 0x00FF000000000000ULL) << 8);
#else
#   error "Unsupported ISA"
#endif
}

Addr
Tcu::nocToPhys(Addr noc)
{
#if THE_ISA == X86_ISA
    return (noc & ~0xFF00000000000000ULL) |
          ((noc & 0xFF00000000000000ULL) >> 16);
#elif THE_ISA == ARM_ISA
    return (noc & ~0xFF00000000000000ULL) |
          ((noc & 0xFF00000000000000ULL) >> 24);
#elif THE_ISA == RISCV_ISA
    return (noc & ~0xFF00000000000000ULL) |
          ((noc & 0xFF00000000000000ULL) >> 8);
#else
#   error "Unsupported ISA"
#endif
}

void
Tcu::startTransfer(void *event, Cycles delay)
{
    auto ev = reinterpret_cast<XferUnit::TransferEvent*>(event);
    xferUnit->startTransfer(ev, delay);
}

size_t
Tcu::startTranslate(vpeid_t vpeId,
                    Addr virt,
                    uint access,
                    bool can_pf,
                    XferUnit::Translation *trans)
{
    // if a command is running, send the response now to finish its memory
    // write instruction to the COMMAND register
    if (cmdPkt)
    {
        stopSleep();
        schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));
        cmdPkt = NULL;
    }

    return coreReqs.startTranslate(vpeId, virt, access, can_pf, trans);
}

size_t
Tcu::startForeignReceive(epid_t epId, vpeid_t vpeId)
{
    // as above
    if (cmdPkt)
    {
        stopSleep();
        schedCpuResponse(cmdPkt, clockEdge(Cycles(1)));
        cmdPkt = NULL;
    }

    return coreReqs.startForeignReceive(epId, vpeId);
}

void
Tcu::abortTranslate(size_t reqId)
{
    coreReqs.abortReq(reqId);
}

void
Tcu::completeNocRequest(PacketPtr pkt)
{
    auto senderState = dynamic_cast<NocSenderState*>(pkt->popSenderState());

    if (senderState->packetType == NocPacketType::CACHE_MEM_REQ)
    {
        // as these target memory PEs, there can't be any error
        assert(senderState->result == TcuError::NONE);

        NocAddr noc(pkt->getAddr());
        DPRINTF(TcuMem,
            "Finished %s request of LLC for %u bytes @ %d:%#x\n",
            pkt->isRead() ? "read" : "write",
            pkt->getSize(), noc.peId, noc.offset);

        if(pkt->isRead())
            printPacket(pkt);

        if (dynamic_cast<InitSenderState*>(pkt->senderState))
        {
            // undo the change from handleCacheMemRequest
            pkt->setAddr(noc.offset - memOffset);
            pkt->req->setPaddr(noc.offset - memOffset);
            pkt->popSenderState();
        }

        // translate NoC address to physical address
        Addr phys = nocToPhys(pkt->getAddr());
        pkt->setAddr(phys);
        pkt->req->setPaddr(phys);

        sendCacheMemResponse(pkt, true);
    }
    else if (senderState->packetType != NocPacketType::CACHE_MEM_REQ_FUNC)
    {
        cmdIsRemote = false;
        // if the current command should be aborted, just ignore the packet
        // and finish the command
        if (abort != AbortType::NONE)
            scheduleFinishOp(Cycles(1), TcuError::ABORT);
        else
        {
            auto cmd = regs().getCommand();
            if (pkt->isWrite())
                memUnit->writeComplete(cmd, pkt, senderState->result);
            else if (pkt->isRead())
                memUnit->readComplete(cmd, pkt, senderState->result);
            else
                panic("unexpected packet type\n");
        }
    }

    delete senderState;
}

void
Tcu::completeMemRequest(PacketPtr pkt)
{
    assert(!pkt->isError());
    assert(pkt->isResponse());

    auto senderState = dynamic_cast<MemSenderState*>(pkt->popSenderState());

    // set the old master id again
    pkt->req->setMasterId(senderState->mid);

    xferUnit->recvMemResponse(senderState->data, pkt);

    delete senderState;
    freeRequest(pkt);
}

void
Tcu::handleNocRequest(PacketPtr pkt)
{
    assert(!pkt->isError());

    if (pkt->cacheResponding())
    {
        DPRINTF(TcuPackets, "Ignoring packet, because cache is responding\n");
        schedNocRequestFinished(clockEdge(Cycles(1)));
        return;
    }

    auto senderState = dynamic_cast<NocSenderState*>(pkt->senderState);

    TcuError res = TcuError::NONE;

    switch (senderState->packetType)
    {
        case NocPacketType::MESSAGE:
        {
            nocMsgRecvs++;
            res = msgUnit->recvFromNoc(pkt);
            break;
        }
        case NocPacketType::READ_REQ:
        case NocPacketType::WRITE_REQ:
        case NocPacketType::CACHE_MEM_REQ:
        {
            if (senderState->packetType == NocPacketType::READ_REQ)
                nocReadRecvs++;
            else if (senderState->packetType == NocPacketType::WRITE_REQ)
                nocWriteRecvs++;
            res = memUnit->recvFromNoc(senderState->tvpe, pkt);
            break;
        }
        case NocPacketType::CACHE_MEM_REQ_FUNC:
            memUnit->recvFunctionalFromNoc(pkt);
            break;
        default:
            panic("Unexpected NocPacketType\n");
    }

    senderState->result = res;
}

bool
Tcu::handleCoreMemRequest(PacketPtr pkt,
                          TcuSlavePort &sport,
                          TcuMasterPort &mport,
                          bool icache,
                          bool functional)
{
    bool res = true;
    bool delayed = false;
    Addr virt = pkt->getAddr();

    DPRINTF(TcuCoreMemAcc, "%s access for %#lx: start\n",
        pkt->cmdString(), virt);

    if (mmioRegion.contains(virt))
    {
        // not supported here
        assert(!functional);

        if (icache)
            res = false;
        else
            forwardRequestToRegFile(pkt, true);
    }
    else
    {
        intMemReqs++;

        if (functional)
            mport.sendFunctional(pkt);
        else
        {
            Tick tick;
            if (cpuToCacheLatency < Cycles(1))
                tick = curTick();
            else
                tick = clockEdge(cpuToCacheLatency);
            mport.schedTimingReq(pkt, tick);
        }
    }

    if (!delayed)
    {
        DPRINTF(TcuCoreMemAcc, "%s access for %#lx: finished\n",
            pkt->cmdString(), virt);
    }

    return res;
}

bool
Tcu::handleCacheMemRequest(PacketPtr pkt, bool functional)
{
    if (pkt->cmd == MemCmd::CleanEvict)
    {
        assert(!pkt->needsResponse());
        DPRINTF(TcuPackets, "Dropping CleanEvict packet\n");
        return true;
    }

    // we don't have cache coherence. so we don't care about invalidate req.
    if (pkt->cmd == MemCmd::InvalidateReq)
        return false;
    if (pkt->cmd == MemCmd::BadAddressError)
        return false;

    Addr physAddr = pkt->getAddr();

    // translate physical address to NoC address
    Addr nocAddr = physToNoc(physAddr);
    pkt->setAddr(nocAddr);

    NocAddr noc(nocAddr);
    // special case: we check whether this is actually a NocAddr. this does
    // only happen when loading a program at startup, TLB misses in the core
    // and pseudoInst
    if (!noc.valid)
    {
        if (noc.offset > memSize)
        {
            DPRINTF(TcuMem, "Ignoring %s request of LLC for %u bytes @ %d:%#x\n",
                pkt->cmdString(), pkt->getSize(), noc.peId, noc.offset);
            return false;
        }

        noc = NocAddr(memPe, memOffset + noc.offset);
        pkt->setAddr(noc.getAddr());
        if (!functional)
        {
            // remember that we did this change
            pkt->pushSenderState(new InitSenderState);
        }
    }

    DPRINTF(TcuMem, "Handling %s request of LLC for %u bytes @ %d:%#x\n",
                    pkt->cmdString(),
                    pkt->getSize(), noc.peId, noc.offset);

    if(pkt->isWrite())
        printPacket(pkt);

    auto type = functional ? Tcu::NocPacketType::CACHE_MEM_REQ_FUNC
                           : Tcu::NocPacketType::CACHE_MEM_REQ;
    // this does always target a memory PE, so vpeId is invalid
    sendNocRequest(type,
                   pkt,
                   INVALID_VPE_ID,
                   Cycles(1),
                   functional);

    extMemReqs++;

    if (functional)
        pkt->setAddr(physAddr);

    return true;
}

void
Tcu::forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest)
{
    Addr oldAddr = pkt->getAddr();

    // Strip the base address to handle requests based on the reg. addr. only.
    pkt->setAddr(oldAddr - mmioRegion.start());

    RegFile::Result result = regFile.handleRequest(pkt, isCpuRequest);

    regFileReqs++;

    // restore old address
    pkt->setAddr(oldAddr);

    if (!atomicMode)
    {
        /*
         * We handle the request immediatly and do not care about timing. The
         * delay is payed by scheduling the response at some point in the
         * future. Additionaly a write operation on the command register needs
         * to schedule an event that executes this command at a future tick.
         */

        Cycles transportDelay =
            ticksToCycles(pkt->headerDelay + pkt->payloadDelay);

        Tick when = clockEdge(transportDelay + registerAccessLatency);

        if (!isCpuRequest)
            schedNocRequestFinished(clockEdge(Cycles(1)));

        if (~result & RegFile::WROTE_EXT_CMD)
        {
            assert(isCpuRequest || result == RegFile::WROTE_NONE);
            pkt->headerDelay = 0;
            pkt->payloadDelay = 0;

            if (isCpuRequest &&
                (result & (RegFile::WROTE_CMD | RegFile::WROTE_PRIV_CMD)) == 0)
                schedCpuResponse(pkt, when);
            else if(!isCpuRequest)
                schedNocResponse(pkt, when);

            if (result & RegFile::WROTE_CMD)
                schedule(new ExecCmdEvent(*this, pkt), when);
            else if (result & RegFile::WROTE_PRIV_CMD)
                schedule(new ExecPrivCmdEvent(*this, pkt), when);
            else if (result & RegFile::WROTE_CORE_REQ)
                schedule(completeCoreReqEvent, when);
            if (result & RegFile::WROTE_PRINT)
                printLine(regs().get(TcuReg::PRINT));
            if (result & RegFile::WROTE_CLEAR_IRQ)
                clearIrq((BaseConnector::IRQ)regs().get(TcuReg::CLEAR_IRQ));
        }
        else
            schedule(new ExecExtCmdEvent(*this, pkt), when);
    }
    else
    {
        if (result & RegFile::WROTE_CMD)
            executeCommand(NULL);
        if (result & RegFile::WROTE_PRIV_CMD)
            executePrivCommand(NULL);
        if (result & RegFile::WROTE_EXT_CMD)
            executeExtCommand(NULL);
        if (result & RegFile::WROTE_CORE_REQ)
            coreReqs.completeReqs();
        if (result & RegFile::WROTE_PRINT)
            printLine(regs().get(TcuReg::PRINT));
        if (result & RegFile::WROTE_CLEAR_IRQ)
            clearIrq((BaseConnector::IRQ)regs().get(TcuReg::CLEAR_IRQ));
    }
}

Tcu*
TcuParams::create()
{
    return new Tcu(this);
}

void
Tcu::printPacket(PacketPtr pkt) const
{
    DPRINTF(TcuPackets, "Dumping packet %s @ %p with %lu bytes\n",
        pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    DDUMP(TcuPackets, pkt->getPtr<uint8_t>(), pkt->getSize());
}
