# Copyright (C) 2015-2018 Nils Asmussen <nils@os.inf.tu-dresden.de>
# Copyright (C) 2019-2020 Nils Asmussen, Barkhausen Institut
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# The views and conclusions contained in the software and documentation are those
# of the authors and should not be interpreted as representing official policies,
# either expressed or implied, of the FreeBSD Project.

from m5.objects.ClockedObject import ClockedObject
from m5.objects.Connector import BaseConnector
from m5.params import *
from m5.proxy import *

class TcuAccel(ClockedObject):
    type = 'TcuAccel'
    abstract = True
    cxx_header = "cpu/tcu-accel/accelerator.hh"
    cxx_class = 'gem5::tcu::TcuAccel'
    port = MasterPort("Port to the TCU and Scratch-Pad-Memory")
    system = Param.System(Parent.any, "System this tester is part of")
    tile_id = Param.Unsigned("Tile ID")
    regfile_base_addr = Param.Addr(0xF0000000, "Register file address")
    offset = Param.Unsigned(0, "The offset that all accesses have to go above")

    max_data_size = Param.MemorySize("1kB", "The maximum size of data transfers")

class TcuAccelConnector(BaseConnector):
    type = 'TcuAccelConnector'
    cxx_header = "cpu/tcu-accel/connector.hh"
    cxx_class = 'gem5::tcu::TcuAccelConnector'

    accelerator = Param.TcuAccel("The accelerator")
