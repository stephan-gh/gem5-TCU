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

#ifndef __MEM_TCU_MSG_UNIT_HH__
#define __MEM_TCU_MSG_UNIT_HH__

#include "mem/tcu/tcu.hh"
#include "mem/tcu/mem_unit.hh"

class MessageUnit
{
  public:

    struct MsgInfo
    {
        epid_t sepId;
        bool ready;
        bool unlimcred;
        uint8_t flags;
        peid_t targetPeId;
        epid_t targetEpId;
        epid_t replyEpId;
        uint64_t label;
        uint64_t replyLabel;
        unsigned replySize;
    };

    class SendTransferEvent : public MemoryUnit::WriteTransferEvent
    {
        MessageUnit *msgUnit;
        MessageHeader *header;

      public:

        SendTransferEvent(MessageUnit *_msgUnit,
                          Addr local,
                          size_t size,
                          uint flags,
                          NocAddr dest,
                          MessageHeader *_header)
            : WriteTransferEvent(local, size, flags, dest),
              msgUnit(_msgUnit),
              header(_header)
        {}

        void transferStart() override;
        void transferDone(TcuError result) override;
    };

    class ReceiveTransferEvent : public MemoryUnit::ReceiveTransferEvent
    {
        MessageUnit *msgUnit;
        Addr msgAddr;

      public:

        ReceiveTransferEvent(MessageUnit *_msgUnit,
                             Addr local,
                             uint flags,
                             PacketPtr pkt)
            : MemoryUnit::ReceiveTransferEvent(
                XferUnit::TransferType::REMOTE_WRITE, local, flags, pkt),
              msgUnit(_msgUnit), msgAddr(local)
        {}

        epid_t rep() const
        {
            NocAddr addr(pkt->getAddr());
            return addr.offset;
        }

        void transferDone(TcuError result) override;
    };

    MessageUnit(Tcu &_tcu) : tcu(_tcu), info() {}

    void regStats();

    /**
     * Start message transmission -> Mem request
     */
    void startTransmission(const CmdCommand::Bits& cmd);

    /**
     * Received a message from NoC -> Mem request
     */
    TcuError recvFromNoc(PacketPtr pkt);

    /**
     * Receives credits again
     */
    void recvCredits(epid_t epid);

    /**
     * Fetches the next message and sets *msgOff to the message offset or -1
     */
    TcuError fetchMessage(epid_t epid, Addr *msgOff);

    /**
     * Acknowledges the message @ <msgOff> within the receive buffer
     */
    TcuError ackMessage(epid_t epId, Addr msgOff);

    /**
     * Finishes a message receive
     */
    TcuError finishMsgReceive(epid_t epId,
                              Addr msgAddr,
                              const MessageHeader *header,
                              TcuError error,
                              uint xferFlags,
                              bool addMsg);

  private:
    int allocSlot(size_t msgSize, epid_t epid, RecvEp *ep);

    void startXfer(const CmdCommand::Bits& cmd);

  private:

    Tcu &tcu;

    MsgInfo info;

    Stats::Histogram sentBytes;
    Stats::Histogram repliedBytes;
    Stats::Histogram receivedBytes;
    Stats::Scalar wrongVPE;
    Stats::Scalar noSpace;

};

#endif
