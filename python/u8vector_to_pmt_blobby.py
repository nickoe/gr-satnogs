#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
from gnuradio import gr
import pmt

class u8vector_to_pmt_blobby(gr.basic_block):
    """
    docstring for block u8vector_to_pmt_blobby
    This is my docstrinc
    """
    def __init__(self, input_arg):
        gr.basic_block.__init__(self,
            name="u8vector_to_pmt_blobby",
            in_sig=None,
            out_sig=None)
        self.message_port_register_out(pmt.intern('msg_out'))
        self.message_port_register_in(pmt.intern('msg_in'))
        self.set_msg_handler(pmt.intern('msg_in'), self.handle_msg)

    def handle_msg(self, msg):
        msg_new = pmt.cdr(msg)
        if not pmt.is_u8vector(msg_new):
            print "[ERROR] Received invalid message type. Expected u8vector"
            return
        self.message_port_pub(pmt.intern('msg_out'), msg_new)
