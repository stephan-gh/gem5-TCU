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

#ifndef __MEM_DTU_DTU_HH__
#define __MEM_DTU_DTU_HH__

#include "mem/dtu/base.hh"
#include "mem/dtu/regfile.hh"
#include "params/Dtu.hh"

class MessageUnit;
class MemoryUnit;

class Dtu : public BaseDtu
{
  public:

    enum MemoryFlags : uint8_t
    {
        READ = (1 << 0),
        WRITE = (1 << 1),
    };

    enum MessageFlags : uint8_t
    {
        REPLY_FLAG = (1 << 0),
        GRANT_CREDITS_FLAG = (1 << 1),
        REPLY_ENABLED = (1 << 2),
    };

    struct MessageHeader
    {
        uint8_t flags; // if bit 0 is set its a reply, if bit 1 is set we grant credits
        uint8_t senderCoreId;
        uint8_t senderEpId;
        uint8_t replyEpId; // for a normal message this is the reply epId
                           // for a reply this is the enpoint that receives credits
        uint16_t length;

        // both should be large enough for pointers.
        uint64_t label;
        uint64_t replyLabel;
    } M5_ATTR_PACKED;

    enum class NocPacketType
    {
        MESSAGE,
        REQUEST,
    };

    enum class SpmPacketType
    {
        FORWARDED_MESSAGE,
        FORWARDED_REQUEST,
        LOCAL_REQUEST,
    };

    struct SpmSenderState : public Packet::SenderState
    {
        SpmPacketType packetType;
        unsigned epId; // only valid if packetType != SpmPacketType::FORWARDER_REUEST
        MasterID mid;
    };

    struct NocSenderState : public Packet::SenderState
    {
        NocPacketType packetType;
    };

    enum class CommandOpcode
    {
        IDLE = 0,
        SEND = 1,
        REPLY = 2,
        READ = 3,
        WRITE = 4,
        INC_READ_PTR = 5,
        WAKEUP_CORE = 6,
    };

    struct Command
    {
        CommandOpcode opcode;
        unsigned epId;
    };

  public:

    static constexpr unsigned numCmdOpcodeBits = 3;

  public:

    Dtu(DtuParams* p);

    ~Dtu();

    RegFile &regs() { return regFile; }
    
    Addr translate(Addr vaddr);
    
    PacketPtr generateRequest(Addr addr, Addr size, MemCmd cmd);
    void freeRequest(PacketPtr pkt);

    Command getCommand();

    void wakeupCore();
    
    void updateSuspendablePin();

    void forwardRequestToRegFile(PacketPtr pkt, bool isCpuRequest);

    void sendFunctionalSpmRequest(PacketPtr pkt) { dcacheMasterPort.sendFunctional(pkt); }

    void scheduleFinishOp(Cycles delay) { schedule(finishCommandEvent, clockEdge(delay)); }

    void sendSpmRequest(PacketPtr pkt,
                        unsigned epId,
                        Cycles delay,
                        SpmPacketType packetType);

    void sendNocRequest(PacketPtr pkt,
                        Cycles delay,
                        bool isMessage);

    void printPacket(PacketPtr pkt) const;

  private:

    void executeCommand();

    void finishCommand();

    void completeLocalSpmRequest(PacketPtr pkt);

    void completeNocRequest(PacketPtr pkt) override;

    void completeSpmRequest(PacketPtr pkt) override;

    void handleNocRequest(PacketPtr pkt) override;

    void handleCpuRequest(PacketPtr pkt) override;

  private:

    const MasterID masterId;

    const bool usePTable;

    System *system;

    RegFile regFile;

    MessageUnit *msgUnit;

    MemoryUnit *memUnit;

    EventWrapper<Dtu, &Dtu::executeCommand> executeCommandEvent;
    
    EventWrapper<Dtu, &Dtu::finishCommand> finishCommandEvent;

  public:

    const bool atomicMode;

    const unsigned numEndpoints;

    const Addr maxNocPacketSize;

    const unsigned numCmdEpidBits;

    const Cycles registerAccessLatency;
    const Cycles commandToSpmRequestLatency;
    const Cycles commandToNocRequestLatency;
    const Cycles spmResponseToNocRequestLatency;
    const Cycles nocMessageToSpmRequestLatency;
    const Cycles nocResponseToSpmRequestLatency;
    const Cycles nocRequestToSpmRequestLatency;
    const Cycles spmResponseToNocResponseLatency;
};

#endif // __MEM_DTU_DTU_HH__
