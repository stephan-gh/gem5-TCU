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

#ifndef __MEM_DTU_PT_WALKER_HH__
#define __MEM_DTU_PT_WALKER_HH__

#include "base/types.hh"
#include "base/bitunion.hh"
#include "sim/eventq.hh"
#include "mem/packet.hh"
#include "mem/dtu/noc_addr.hh"
#include "mem/dtu/tlb.hh"

class Dtu;

class PtUnit
{
  public:

    struct Translation
    {
        Translation()
        {}
        virtual ~Translation()
        {}

        virtual void finished(bool success, const NocAddr &phys) = 0;
    };

    BitUnion64(PageTableEntry)
        Bitfield<63, DtuTlb::PAGE_BITS> base;
        Bitfield<2,0> xwr;
        Bitfield<2> x;
        Bitfield<1> w;
        Bitfield<0> r;
    EndBitUnion(PageTableEntry)

  private:

    struct TranslateEvent : public Event
    {
        PtUnit& unit;

        Addr virt;
        DtuTlb::Flag access;
        Translation *trans;

        TranslateEvent(PtUnit& _unit)
            : unit(_unit), virt(), access(), trans()
        {}

        void process() override;

        void recvFromMem(PacketPtr pkt);

        const char* description() const override { return "TranslateEvent"; }

        const std::string name() const override;
    };

  public:
    
    PtUnit(Dtu& _dtu) : dtu(_dtu)
    {}

    bool translateFunctional(Addr virt, DtuTlb::Flag access, NocAddr *phys);

    void startTranslate(Addr virt, DtuTlb::Flag access, Translation *trans);

    void recvFromMem(Addr event, PacketPtr pkt)
    {
        TranslateEvent *ev = reinterpret_cast<TranslateEvent*>(event);
        ev->recvFromMem(pkt);
    }

  private:

    PacketPtr createPacket(Addr virt);
    
    bool finishTranslate(PacketPtr pkt, Addr virt, DtuTlb::Flag access, NocAddr *phys);

    Dtu& dtu;

};

#endif