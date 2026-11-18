package nulctrl

import chisel3._
import chisel3.util._

// Based on the FASE v4 control-ROM implementation at e6a3e5a.
// The BOOM variant shares the page SRAM ports and prefetches write data.
private object V4_CPU_State {
  val CPU_INIT = 0.U
  val CPU_HALT = 1.U
  val CPU_ITR  = 2.U
  val CPU_USER = 3.U
}

private object V4_SER_OP {
  val SEROP_NEXT  = 0.U   //  OP[8]                               -> ACK[8] ID[8] CAUSE[8] PC[48] (ARG[48])
  val SEROP_HALT  = 1.U   //  OP[8]       ID[8]                  -> ACK[8]
  val SEROP_ITR   = 2.U   //  OP[8]       ID[8]                  -> ACK[8]
  val SEROP_MMU   = 3.U   //  OP[8]       ID[8] ASID[16] PT[40]  -> ACK[8]
  val SEROP_REDIR = 4.U   //  OP[8]       ID[8] PC[48]           -> ACK[8]
  val SEROP_FTLB  = 5.U   //  OP[8]       ID[8]                  -> ACK[8]
  val SEROP_FTLB2 = 6.U   //  OP[8]       ID[8] ASID[16] VPN[40] -> ACK[8]
  val SEROP_SYNCI = 7.U   //  OP[8]       ID[8]                  -> ACK[8]
  val SEROP_REGRD = 8.U   //  OP[8]       ID[8] REG[16]          -> ACK[8] DATA[64]
  val SEROP_REGWT = 9.U   //  OP[8]       ID[8] REG[16] DATA[64] -> ACK[8]
  val SEROP_MEMRD = 10.U  //  OP[8]       ID[8] PA[48]           -> ACK[8] DATA[64]
  val SEROP_MEMWT = 11.U  //  OP[8]       ID[8] PA[48] DATA[64]  -> ACK[8]
  val SEROP_PGRD  = 12.U  //  OFF[3]OP[5] ID[8] PPN[40]          -> ACK[8] DATA[512B]
  val SEROP_PGST  = 13.U  //  OP[8]       ID[8] PPN[40] VALUE[64]-> ACK[8]
  val SEROP_PGWT  = 14.U  //  OFF[3]OP[5] ID[8] PPN[40]          DATA[512B]-> ACK[8]
  val SEROP_PGCP  = 15.U  //  OP[8]       ID[8] DST[40] SRC[40]  -> ACK[8]
  val SEROP_CLK   = 16.U  //  OP[8]                               -> ACK[8] CLK[64]
  val SEROP_UCLK  = 17.U  //  OP[8]       ID[8]                  -> ACK[8] CLK[64]
  val SEROP_HFSET = 18.U  //  OP[8]       ID[8] VA[48]           -> ACK[8]
  val SEROP_HFCLR = 19.U  //  OP[8]       ID[8]                  -> ACK[8]
  val SEROP_PGZERO = 20.U //  OP[8]       ID[8] PPN[40]          -> ACK[8]
  val SEROP_INST  = 30.U
}

private object V4_Ctrl_State {
  val STATE_INIT_WAIT = 0.U
  val STATE_RECV_HEAD = 1.U
  val STATE_RECV_ARG = 2.U
  val STATE_DO_OP = 3.U
  val STATE_Jump_HFUTEX = 4.U
  val STATE_RUN = 5.U
  val STATE_SEND_HEAD = 6.U
  val STATE_SEND_ARG = 7.U
  val STATE_WAIT_NEXT = 8.U
  val STATE_LOAD_DATA = 9.U
  val STATE_DUMP_DATA = 10.U
  val STATE_ERROR = 11.U
}
private object V4_OP_OP {
  val STATE_DO_INIT       = 0.U
  val STATE_NEXT          = 2.U
  val STATE_MMU           = 3.U
  val STATE_REDIR         = 4.U
  val STATE_FLUSH         = 5.U
  val STATE_FLUSH2        = 6.U
  val STATE_REGRD         = 7.U
  val STATE_REGWT         = 8.U
  val STATE_MEMRD         = 9.U
  val STATE_MEMWT         = 10.U
  val STATE_PGRD          = 11.U
  val STATE_PGST          = 12.U
  val STATE_PGWT          = 13.U
  val STATE_PGCP          = 14.U
  val STATE_SYNCI         = 15.U
  val STATE_INST          = 16.U
  val STATE_HFUTEX        = 17.U
}


class NulCPUCtrlMPMicrocode(cpunum: Int) extends Module {
  val io = IO(new FaseControllerIO(cpunum))

  val is_sv48 = false
  val rom_size = 256
  val rom_addr_wid = 8
  val RV_ITR_ECALL = 8.U
  val ECALL_FUTEX = 98.U
  assert(cpunum > 0)
  val cpunum_bitwid = if (cpunum > 1) {
    log2Ceil(cpunum)
  } else {
    1
  }

  io.rx.ready := false.B
  io.tx.valid := false.B
  io.tx.bits := 0.U

  for (i <- 0 until cpunum) {
    io.cpu(i).ext_itr := false.B
    io.cpu(i).regacc_rd := false.B
    io.cpu(i).regacc_wt := false.B
    io.cpu(i).regacc_idx := 0.U
    io.cpu(i).regacc_wdata := 0.U
    io.cpu(i).inst64 := false.B
    io.cpu(i).inst64_raw := 0.U
    io.cpu(i).inst64_nowait := false.B
    io.cpu(i).inst64_flush := false.B
  }

  val cpu_state = RegInit(VecInit(Seq.fill(cpunum)(0.U(2.W))))

  val global_clk = RegInit(0.U(64.W))
  val user_clk = RegInit(VecInit(Seq.fill(cpunum)(0.U(64.W))))

  global_clk := global_clk + 1.U
  for (i <- 0 until cpunum) {
    when(io.cpu(i).priv === 0.U) {
      user_clk(i) := user_clk(i) + 1.U
    }
  }

  val cpu_raised_itr = RegInit(VecInit(Seq.fill(cpunum)(false.B)))
  val last_priv = RegInit(VecInit(Seq.fill(cpunum)(3.U(2.W))))
  for (i <- 0 until cpunum) {
    last_priv(i) := io.cpu(i).priv
    when((last_priv(i) === 0.U) && (io.cpu(i).priv =/= 0.U) && (cpu_state(i) === V4_CPU_State.CPU_USER)) {
      cpu_raised_itr(i) := true.B
      cpu_state(i) := V4_CPU_State.CPU_ITR
    }
    io.cpu(i).stop_fetch := (cpu_state(i) === V4_CPU_State.CPU_ITR) || (cpu_state(i) === V4_CPU_State.CPU_HALT) || (last_priv(i) === 0.U && io.cpu(i).priv =/= 0.U)
  }
  val event_queue = Module(new Queue(UInt(cpunum_bitwid.W), cpunum + 1))
  val has_itr = cpu_raised_itr.reduce(_ || _)
  val event_idx = PriorityEncoder(cpu_raised_itr.asUInt)
  event_queue.io.enq.valid := has_itr
  event_queue.io.enq.bits := event_idx
  when(has_itr && event_queue.io.enq.ready) {
    cpu_raised_itr(event_idx) := false.B
  }
  /**
   * V4_Ctrl_State
   */
  val state = RegInit(0.U(5.W))
  val trans_bytes = RegInit(0.U(5.W))
  val trans_pos = RegInit(0.U(4.W))
  /**
   * register op what we are operating
   */
  val state_register = RegInit(0.U(5.W))
  val errno = RegInit(0.U(6.W))
  val initialized = RegInit(false.B)
  val errored = (state === V4_Ctrl_State.STATE_ERROR)
  io.dbg_sta := Cat(initialized, errored, errno(5, 0))
  val opcode = RegInit(0.U(5.W))
  val opoff = RegInit(0.U(3.W))
  val oparg = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))
  val retarg = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))
  val opidx = oparg(1)(cpunum_bitwid - 1, 0)

  val ctrl_rom = SyncReadMem(rom_size, UInt(39.W))
  private val romFile = new java.io.File(
    "generators/riscv-boom/src/main/resources/fase_ctrl_v4.hex").getCanonicalPath
  chisel3.util.experimental.loadMemoryFromFileInline(ctrl_rom, romFile)

  val ctrl_ptr = WireDefault(0.U(rom_addr_wid.W))
  val ctrl_content = ctrl_rom.read(ctrl_ptr)
  val ctrl_base_addr = RegInit(0.U(rom_addr_wid.W))
  val ctrl_misc = ctrl_content(31, 0)
  val ctrl_inst_raw = WireDefault(0.U(32.W))
  val ctrl_itr = ctrl_content(38)
  val ctrl_regacc_rd = ctrl_content(37)
  val ctrl_regacc_wt = ctrl_content(36)
  val ctrl_inst64 = ctrl_content(35)
  val ctrl_inst64_nowait = ctrl_content(34)
  val ctrl_inst64_flush = ctrl_content(33)
  val ctrl_last = ctrl_content(32)
  val ctrl_idx = ctrl_content(31, 27) // 5 bits
  val ctrl_header = ctrl_content(26,23) //4 bits
  val ctrl_loc_idx = ctrl_content(22,19) //4 bits
  val cpu_regacc_rdata = io.cpu(opidx).regacc_rdata
  val cpu_regacc_busy = io.cpu(opidx).regacc_busy
  val cpu_inst64_ready = io.cpu(opidx).inst64_ready
  val cpu_inst64_busy = io.cpu(opidx).inst64_busy
  val regacc_wdata_ptr = io.cpu(opidx).regacc_wdata
  val regacc_idx_wire = WireDefault(0.U(5.W))

  val all_inited = io.cpu.map(_.inited).reduce(_ && _)
  val inited_num = RegInit(0.U(9.W))
  val cnt_run = RegInit(0.U(6.W)) //clear each time goto state_run
  val hfutex_masks = RegInit(VecInit(Seq.fill(cpunum)(VecInit(Seq.fill(4)(0.U(48.W))))))
  val hfutex_pos = RegInit(VecInit(Seq.fill(cpunum)(0.U(2.W))))
  val hfutex_match_reg = RegInit(0.U(48.W))
  val hfutex_set_value = Cat(oparg(7), oparg(6), oparg(5), oparg(4), oparg(3), oparg(2))
  val hfutex_hit = hfutex_masks(opidx).map(_ === hfutex_match_reg).reduce(_ || _)
  val hfutex_hit_set = hfutex_masks(opidx).map(_ === hfutex_set_value).reduce(_ || _)
  val all_halted = cpu_state.forall(_ === V4_CPU_State.CPU_HALT)
  val regback = RegInit(VecInit(Seq.fill(10)(0.U(64.W))))
  val pg_base_addr2 = Cat((0.U(12.W)), oparg(6), oparg(5), oparg(4), oparg(3), oparg(2), (0.U(12.W)))

  val pg_base_addr7 = Cat((0.U(12.W)), oparg(11), oparg(10), oparg(9), oparg(8), oparg(7), (0.U(12.W)))

  val pg_loop_cnt = RegInit(0.U(8.W))

  val pg_addr2_plus = ( pg_base_addr2 | (opoff << 9).asUInt).asUInt

  val page_mem = SyncReadMem(64,Vec(8,UInt(8.W)))
  val page_ptr = WireDefault(0.U(6.W))
  val page_ptr_in_rw = RegInit(0.U(7.W))
  val page_write_mask = Wire(Vec(8,Bool()))
  val byte_cnt = RegInit(0.U(9.W))
  val page_word_content = Wire(Vec(8,UInt(8.W)))
  for(index <- 0 to 7){
    page_write_mask(index)  := false.B
    page_word_content(index) := 0.U
  }
  val pageReadEnable = WireDefault(false.B)
  val pageReadAddress = WireDefault(0.U(6.W))
  val pageReadData = page_mem.read(pageReadAddress, pageReadEnable)
  val capturePageWord = WireDefault(false.B)

  event_queue.io.deq.ready := (state === V4_Ctrl_State.STATE_WAIT_NEXT)
  when(state === V4_Ctrl_State.STATE_WAIT_NEXT && event_queue.io.deq.valid) {
    val pending_next_idx = event_queue.io.deq.bits.pad(8)
    oparg(1) := pending_next_idx // to update sel_cpu port
    retarg(0) := pending_next_idx
    state_register := V4_OP_OP.STATE_NEXT
    state := V4_Ctrl_State.STATE_RUN
    cnt_run := 0.U
    ctrl_base_addr := 8.U
    ctrl_ptr := 8.U
  }.elsewhen(state === V4_Ctrl_State.STATE_WAIT_NEXT && all_halted) {
    retarg(0) := "hff".U
    state := V4_Ctrl_State.STATE_SEND_HEAD
  }
  //state machine, two-process FSM
  //next-state and sequential logic
  //begin from wait_init
  when(state === V4_Ctrl_State.STATE_INIT_WAIT){
    when(all_inited){
      when(inited_num < cpunum.U){
        state_register := V4_OP_OP.STATE_DO_INIT
        oparg(1) := inited_num(cpunum_bitwid - 1, 0) // when cpunum < 255, it must works
        state := V4_Ctrl_State.STATE_RUN
        cnt_run := 0.U
        for(index <- 0 until cpunum){
          cpu_state(index) := V4_CPU_State.CPU_HALT
        }
      }.elsewhen(inited_num === cpunum.U) {
        initialized := true.B
        state := V4_Ctrl_State.STATE_RECV_HEAD
      }
    }
  }
  val will_next = WireDefault(false.B)

  when(state === V4_Ctrl_State.STATE_RECV_HEAD) {
    io.rx.ready := true.B
    when(io.rx.valid) {
      val rxop = io.rx.bits(4, 0)
      val rxoff = io.rx.bits(7, 5)
      opcode := rxop
      opoff := rxoff
      trans_pos := 1.U
      state := V4_Ctrl_State.STATE_RECV_ARG

      when(rxop === V4_SER_OP.SEROP_HALT || rxop === V4_SER_OP.SEROP_ITR || rxop === V4_SER_OP.SEROP_FTLB || rxop === V4_SER_OP.SEROP_SYNCI || rxop === V4_SER_OP.SEROP_UCLK || rxop === V4_SER_OP.SEROP_HFCLR) {
        trans_bytes := 2.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_REGRD) {
        trans_bytes := 4.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_INST) {
        trans_bytes := 6.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_PGRD || rxop === V4_SER_OP.SEROP_PGWT || rxop === V4_SER_OP.SEROP_PGZERO) {
        trans_bytes := 7.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_REDIR || rxop === V4_SER_OP.SEROP_MEMRD || rxop === V4_SER_OP.SEROP_HFSET) {
        trans_bytes := 8.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_MMU || rxop === V4_SER_OP.SEROP_FTLB2) {
        trans_bytes := 9.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_REGWT || rxop === V4_SER_OP.SEROP_PGCP) {
        trans_bytes := 12.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_PGST) {
        trans_bytes := 15.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_MEMWT) {
        trans_bytes := 16.U
      }.elsewhen(rxop === V4_SER_OP.SEROP_NEXT || rxop === V4_SER_OP.SEROP_CLK) {
        state := V4_Ctrl_State.STATE_DO_OP
        trans_bytes := 1.U
      }.otherwise {
        state := V4_Ctrl_State.STATE_ERROR
        errno := rxop
      }
    }
  }
  val page_mem_can_write = WireDefault(false.B)
  when(state === V4_Ctrl_State.STATE_LOAD_DATA){
    io.rx.ready := true.B
    for(index <- 0 until 8){
      page_word_content(index) := io.rx.bits
    }
    page_ptr := byte_cnt(8,3)
    page_write_mask := UIntToOH(byte_cnt(2,0),8).asBools
    when(io.rx.fire){
      page_mem_can_write := true.B
      when(byte_cnt === 511.U){
        byte_cnt := 0.U
        state := V4_Ctrl_State.STATE_DO_OP
      }.otherwise{
        byte_cnt := byte_cnt + 1.U
      }
    }
  }
  // UART reception and CPU capture are mutually exclusive and share one SRAM write port.
  when(page_mem_can_write || capturePageWord) {
    val address = Mux(capturePageWord, page_ptr_in_rw(5, 0), page_ptr)
    val data = Mux(capturePageWord, cpu_regacc_rdata.asTypeOf(Vec(8, UInt(8.W))), page_word_content)
    val mask = Mux(capturePageWord, VecInit(Seq.fill(8)(true.B)), page_write_mask)
    page_mem.write(address, data, mask.toSeq)
  }


  when(state === V4_Ctrl_State.STATE_RECV_ARG) {
    io.rx.ready := true.B
    when(io.rx.valid) {
      oparg(trans_pos) := io.rx.bits
      when(trans_pos +& 1.U === trans_bytes) {
        trans_pos := 0.U
        trans_bytes := 0.U
        state := V4_Ctrl_State.STATE_DO_OP
        when(opcode === V4_SER_OP.SEROP_PGWT){
          state := V4_Ctrl_State.STATE_LOAD_DATA
          byte_cnt := 0.U
        }
      }.otherwise {
        trans_pos := trans_pos + 1.U
      }
    }
  }

  when(state === V4_Ctrl_State.STATE_DO_OP) {
    trans_bytes := 1.U
    trans_pos := 1.U
    state := V4_Ctrl_State.STATE_SEND_HEAD
    switch(opcode) {
      is(V4_SER_OP.SEROP_NEXT) {
        state := V4_Ctrl_State.STATE_WAIT_NEXT
      }
      is(V4_SER_OP.SEROP_HALT) {
        io.cpu(opidx).ext_itr := true.B
        cpu_state(opidx) := V4_CPU_State.CPU_HALT
      }
      is(V4_SER_OP.SEROP_ITR) {
        io.cpu(opidx).ext_itr := true.B
      }
      is(V4_SER_OP.SEROP_MMU) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_MMU
        ctrl_base_addr := 43.U
        ctrl_ptr := 43.U
        cnt_run := 0.U
      }
      is(V4_SER_OP.SEROP_REDIR) {
        when(cpu_state(opidx) === V4_CPU_State.CPU_HALT || cpu_state(opidx) === V4_CPU_State.CPU_ITR) {
          state := V4_Ctrl_State.STATE_RUN
          cnt_run := 0.U
          state_register := V4_OP_OP.STATE_REDIR
          ctrl_base_addr := 75.U
          ctrl_ptr := 75.U
        }
      }
      is(V4_SER_OP.SEROP_FTLB) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_FLUSH
        cnt_run := 0.U
        ctrl_base_addr := 50.U
        ctrl_ptr := 50.U
      }
      is(V4_SER_OP.SEROP_FTLB2) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_FLUSH2
        ctrl_base_addr := 52.U
        ctrl_ptr := 52.U
        cnt_run := 0.U
      }
      is(V4_SER_OP.SEROP_SYNCI) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_SYNCI
        cnt_run := 0.U
        ctrl_base_addr := 61.U
        ctrl_ptr := 61.U
      }
      is(V4_SER_OP.SEROP_REGRD) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_REGRD
        cnt_run := 0.U
        when(oparg(2)(5)) {
          ctrl_base_addr := 64.U
          ctrl_ptr := 64.U
        }.otherwise {
          ctrl_base_addr := 63.U
          ctrl_ptr := 63.U
        }
      }
      is(V4_SER_OP.SEROP_REGWT) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_REGWT
        cnt_run := 0.U
        when(oparg(2)(5)) {
          ctrl_base_addr := 70.U
          ctrl_ptr := 70.U
        }.otherwise {
          ctrl_base_addr := 69.U
          ctrl_ptr := 69.U
        }
      }
      is(V4_SER_OP.SEROP_MEMRD) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_MEMRD
        cnt_run := 0.U
        ctrl_base_addr := 92.U
        ctrl_ptr := 92.U
      }
      is(V4_SER_OP.SEROP_MEMWT) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_MEMWT
        cnt_run := 0.U
        ctrl_base_addr := 101.U
        ctrl_ptr := 101.U
      }
      is(V4_SER_OP.SEROP_CLK) {
        for (i <- 0 to 7) {
          retarg(i) := global_clk(i * 8 + 7, i * 8)
        }
      }
      is(V4_SER_OP.SEROP_UCLK) {
        for (i <- 0 to 7) {
          retarg(i) := user_clk(opidx)(i * 8 + 7, i * 8)
        }
      }
      is(V4_SER_OP.SEROP_HFSET) {
        when(!hfutex_hit_set) {
          hfutex_masks(opidx)(hfutex_pos(opidx)) := hfutex_set_value
          hfutex_pos(opidx) := Mux(hfutex_pos(opidx) === 3.U, 0.U, hfutex_pos(opidx) + 1.U)
        }
      }
      is(V4_SER_OP.SEROP_HFCLR) {
        for (i <- 0 to 3) {
          hfutex_masks(opidx)(i) := 0.U
        }
      }
      is(V4_SER_OP.SEROP_INST) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_INST
        cnt_run := 0.U
        ctrl_base_addr := 48.U
        ctrl_ptr := 48.U
      }
      is(V4_SER_OP.SEROP_PGRD) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_PGRD
        cnt_run := 0.U
        ctrl_base_addr := 173.U
        ctrl_ptr := 173.U
        page_ptr_in_rw := 0.U
      }
      is(V4_SER_OP.SEROP_PGST) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_PGST
        cnt_run := 0.U
        ctrl_base_addr := 110.U
        ctrl_ptr := 110.U
      }
      is(V4_SER_OP.SEROP_PGZERO) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_PGST
        cnt_run := 0.U
        ctrl_base_addr := 110.U
        ctrl_ptr := 110.U
        for (index <- 0 to 7) {
          oparg(7 + index) := 0.U
        }
      }
      is(V4_SER_OP.SEROP_PGWT) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_PGWT
        cnt_run := 0.U
        ctrl_base_addr := 214.U
        ctrl_ptr := 214.U
        page_ptr_in_rw := 0.U
      }
      is(V4_SER_OP.SEROP_PGCP) {
        state := V4_Ctrl_State.STATE_RUN
        state_register := V4_OP_OP.STATE_PGCP
        cnt_run := 0.U
        ctrl_base_addr := 129.U
        ctrl_ptr := 129.U
      }
    }
  }
  when(state === V4_Ctrl_State.STATE_SEND_HEAD) {
    io.tx.valid := true.B
    io.tx.bits := Cat(opoff, opcode)
    trans_pos := 0.U
    when(io.tx.ready) {
      when(opcode === V4_SER_OP.SEROP_NEXT) {
        trans_bytes := 14.U
        state := V4_Ctrl_State.STATE_SEND_ARG
      }.elsewhen(opcode === V4_SER_OP.SEROP_REGRD || opcode === V4_SER_OP.SEROP_MEMRD || opcode === V4_SER_OP.SEROP_CLK || opcode ===  V4_SER_OP.SEROP_UCLK) {
        trans_bytes := 8.U
        state := V4_Ctrl_State.STATE_SEND_ARG
      }.elsewhen(opcode === V4_SER_OP.SEROP_PGRD){
        state := V4_Ctrl_State.STATE_DUMP_DATA
        byte_cnt := 0.U
        pageReadEnable := true.B
      }.otherwise {
        trans_bytes := 0.U
        state := V4_Ctrl_State.STATE_RECV_HEAD
      }
    }
  }

  when(state === V4_Ctrl_State.STATE_SEND_ARG) {
    io.tx.valid := true.B
    io.tx.bits := retarg(trans_pos)
    when(io.tx.ready) {
      when(trans_pos +& 1.U === trans_bytes) {
        trans_pos := 0.U
        trans_bytes := 0.U
        state := V4_Ctrl_State.STATE_RECV_HEAD
      }.otherwise {
        trans_pos := trans_pos + 1.U
      }
    }
  }
  when(state === V4_Ctrl_State.STATE_DUMP_DATA) {
    io.tx.valid := true.B
    pageReadEnable := true.B
    // Re-read the current word during backpressure. Select the next word
    // only when the transmitter actually consumes the boundary byte.
    pageReadAddress := (byte_cnt + io.tx.fire.asUInt)(8, 3)
    io.tx.bits := pageReadData(byte_cnt(2, 0))
    when(io.tx.fire){
      when(byte_cnt === 511.U){
        byte_cnt := 0.U
        state := V4_Ctrl_State.STATE_RECV_HEAD
      }.otherwise{
        byte_cnt := byte_cnt + 1.U
      }
    }
  }
//default just next, not jump back
  val jump_tap = WireDefault(false.B)
  val jump_condition = WireDefault(false.B)
  val cnt_rerun = WireDefault(0.U(6.W))
  val no_wait_tap = state === V4_Ctrl_State.STATE_RUN && ctrl_inst64_nowait && ctrl_inst64
  // The core leaves the monitor on MRET; it cannot report idle until its next trap.
  val postMret = (state_register === V4_OP_OP.STATE_REDIR && cnt_run === 16.U) ||
    (state_register === V4_OP_OP.STATE_HFUTEX && cnt_run === 17.U)
    switch(state_register){
      is(V4_OP_OP.STATE_PGST){
        cnt_rerun := 4.U
        jump_tap := cnt_run === 14.U
        jump_condition := pg_loop_cnt =/= 63.U
      }
      is(V4_OP_OP.STATE_PGCP){
        cnt_rerun := 12.U
        jump_tap := cnt_run === 31.U
        jump_condition := pg_loop_cnt =/= 63.U
      }
      is(V4_OP_OP.STATE_PGRD){
        cnt_rerun := 11.U
        jump_tap := cnt_run === 29.U
        jump_condition := page_ptr_in_rw =/= 64.U
      }
      is(V4_OP_OP.STATE_PGWT){
        cnt_rerun := 10.U
        jump_tap := cnt_run === 29.U
        jump_condition := page_ptr_in_rw =/= 64.U
      }
      is(V4_OP_OP.STATE_HFUTEX){
        jump_tap := cnt_run === 2.U
        val no_jump_should = regback(1)(6,0) === 1.U && hfutex_match_reg =/= 0.U && hfutex_hit
        jump_condition := !no_jump_should
        cnt_rerun := 0.U
      }
    }
  when(no_wait_tap){
    when(cpu_inst64_ready){
      cpu_state(opidx) := V4_CPU_State.CPU_USER
    }
  }
    val cnt_can_jump = WireDefault(false.B)
    cnt_can_jump := jump_tap && jump_condition
    //state change according to state_run
    when(state === V4_Ctrl_State.STATE_RUN){
      //every time will_next is high update cnt
      //every time oplast is high switch to next state
      when(will_next) {
        when(ctrl_last) {
          when(state_register === V4_OP_OP.STATE_DO_INIT) {
            state := V4_Ctrl_State.STATE_INIT_WAIT
            inited_num := inited_num + 1.U
          }.elsewhen(state_register === V4_OP_OP.STATE_NEXT){
            when(retarg(1) === RV_ITR_ECALL ){
              for(index <- 0 to 5){
                retarg(index +8) := regback(0)(index * 8 + 7, index * 8)
              }
              when(regback(0) === ECALL_FUTEX){
                state := V4_Ctrl_State.STATE_Jump_HFUTEX
              }.otherwise{
                state := V4_Ctrl_State.STATE_SEND_HEAD
              }
            }.otherwise{
              state := V4_Ctrl_State.STATE_SEND_HEAD
            }
          }.elsewhen(state_register === V4_OP_OP.STATE_HFUTEX){
            state := V4_Ctrl_State.STATE_WAIT_NEXT
          }.otherwise {
            state := V4_Ctrl_State.STATE_SEND_HEAD
          }
        }
      }
    }
  when(state === V4_Ctrl_State.STATE_Jump_HFUTEX){
    state_register := V4_OP_OP.STATE_HFUTEX
    state := V4_Ctrl_State.STATE_RUN
    cnt_run := 0.U
    ctrl_base_addr := 23.U
    ctrl_ptr := 23.U
  }

  //output logic
  when(state === V4_Ctrl_State.STATE_RUN){
    when(ctrl_regacc_wt || ctrl_regacc_rd){
      will_next := !cpu_regacc_busy
    }.elsewhen(ctrl_inst64){
      will_next := cpu_inst64_ready
    }.elsewhen(ctrl_inst64_flush){
      will_next := !cpu_inst64_busy || postMret
    }.otherwise({
      will_next := true.B
    })
  }
  when(state === V4_Ctrl_State.STATE_RUN){
    ctrl_ptr := ctrl_base_addr + cnt_run + will_next
    when(will_next){
      cnt_run := cnt_run + 1.U
      when(jump_tap){
        when(jump_condition){
          ctrl_ptr := ctrl_base_addr + cnt_rerun
          cnt_run := cnt_rerun
          when(state_register === V4_OP_OP.STATE_PGST || state_register === V4_OP_OP.STATE_PGCP ){
            pg_loop_cnt := pg_loop_cnt + 1.U
          }
          when(state_register === V4_OP_OP.STATE_HFUTEX){
            //jump out
            state := V4_Ctrl_State.STATE_SEND_HEAD
          }
        }.elsewhen(!jump_condition){
          //go to next
          ctrl_ptr := ctrl_base_addr + cnt_run + 1.U
          cnt_run := cnt_run + 1.U
          when(state_register === V4_OP_OP.STATE_PGST || state_register === V4_OP_OP.STATE_PGCP ){
            pg_loop_cnt := 0.U
          }.elsewhen((state_register === V4_OP_OP.STATE_PGRD) || (state_register === V4_OP_OP.STATE_PGWT)) {
            page_ptr_in_rw := 0.U
          }
        }
      }
    }
  }


  when(state === V4_Ctrl_State.STATE_RUN) {
    io.cpu(opidx).ext_itr := ctrl_itr
    io.cpu(opidx).regacc_rd := ctrl_regacc_rd
    io.cpu(opidx).regacc_wt := ctrl_regacc_wt
    io.cpu(opidx).regacc_idx := regacc_idx_wire
    io.cpu(opidx).inst64 := ctrl_inst64
    io.cpu(opidx).inst64_nowait := ctrl_inst64_nowait
    io.cpu(opidx).inst64_flush := ctrl_inst64_flush
    io.cpu(opidx).inst64_raw := ctrl_inst_raw
  }
  //WAIT_INIT and WAIT_NEXT don't need load stage
  when(state === V4_Ctrl_State.STATE_INIT_WAIT){
    ctrl_base_addr := 0.U
    ctrl_ptr := 0.U
  }
  when(state === V4_Ctrl_State.STATE_WAIT_NEXT){
    ctrl_base_addr := 8.U
    ctrl_ptr := 8.U
  }


  val def_pmp_cfg = ("h1f".U(64.W))
  val def_pmp_addr = if(is_sv48) ("hffffffffffff".U(64.W)) else ("h7fffffffff".U(64.W))
  val satp_mode = if(is_sv48) 9.U(4.W) else 8.U(4.W)
  val satp_value = Cat(
    satp_mode,
    oparg(3),
    oparg(2),
    (0.U(4.W)),
    oparg(8),
    oparg(7),
    oparg(6),
    oparg(5),
    oparg(4)
  )
  val flush_tlb_address = Cat((0.U(12.W)), oparg(8), oparg(7), oparg(6), oparg(5), oparg(4), (0.U(12.W)))
  val flush_asid = Cat((0.U(48.U)), oparg(3), oparg(2))




  when(state === V4_Ctrl_State.STATE_RUN && ctrl_regacc_rd && !cpu_regacc_busy) {
    switch(ctrl_header){
      is("b0000".U){
        regback(ctrl_loc_idx) := cpu_regacc_rdata
      }
      is("b0001".U){
        hfutex_match_reg := cpu_regacc_rdata
      }
      is("b0100".U){
        retarg(1) := cpu_regacc_rdata(7, 0)
      }
      is("b0101".U){
        for(index <- 0 to 5){
          val value = cpu_regacc_rdata(index * 8 + 7, index * 8)
          when(ctrl_loc_idx === 2.U) {
            retarg(2 + index) := value
          }.otherwise {
            retarg(8 + index) := value
          }
        }
      }
      is("b0110".U){
        for(index <- 0 to 7){
          retarg(index) := cpu_regacc_rdata(index * 8 + 7, index * 8)
        }
      }
    }
  }
  when(ctrl_regacc_wt){
    switch(ctrl_header){
      is("b0000".U){
        regacc_wdata_ptr := def_pmp_cfg
      }
      is("b0001".U){
        regacc_wdata_ptr := def_pmp_addr
      }
      is("b0010".U){
        regacc_wdata_ptr := regback(ctrl_loc_idx)
      }
      is("b0011".U){
        regacc_wdata_ptr := satp_value
      }
      is("b0100".U){
        regacc_wdata_ptr := flush_tlb_address
      }
      is("b0101".U) {
        regacc_wdata_ptr := flush_asid
      }
      is("b0110".U){
        // The ROM selects one of the three 64-bit UART arguments.
        val regValue = Cat(oparg.slice(4, 12).reverse)
        val fillValue = Cat(oparg.slice(7, 15).reverse)
        val memValue = Cat(oparg.slice(8, 16).reverse)
        regacc_wdata_ptr := Mux(ctrl_loc_idx === 4.U, regValue,
          Mux(ctrl_loc_idx === 7.U, fillValue, memValue))
      }
      is("b0111".U){
        regacc_wdata_ptr := Cat(0.U(16.W), Cat(oparg.slice(2, 8).reverse))
      }
      is("b1000".U){
        regacc_wdata_ptr := pg_base_addr2
      }
      is("b1001".U) {
        regacc_wdata_ptr := pg_base_addr7
      }
      is("b1010".U){
        regacc_wdata_ptr := pageReadData.asUInt
      }
      is("b1011".U){
        regacc_wdata_ptr := pg_addr2_plus
      }
    }
  }
  regacc_idx_wire := ctrl_idx
  when((state_register === V4_OP_OP.STATE_REGRD || state_register=== V4_OP_OP.STATE_REGWT  )&& !oparg(2)(5)){
    regacc_idx_wire := oparg(2)(4,0)
  }.otherwise{
    regacc_idx_wire := ctrl_idx
  }
  ctrl_inst_raw := ctrl_misc
  when(state_register === V4_OP_OP.STATE_REGWT){
    ctrl_inst_raw := (ctrl_misc | (oparg(2)(4,0) << 7 ).asUInt)
  }.elsewhen(state_register === V4_OP_OP.STATE_REGRD){
    ctrl_inst_raw := (ctrl_misc | (oparg(2)(4,0) << 15 ).asUInt)
  }.elsewhen(state_register === V4_OP_OP.STATE_INST){
    ctrl_inst_raw := Cat(oparg(5), oparg(4), oparg(3), oparg(2))
  }.otherwise{
    ctrl_inst_raw := ctrl_misc
  }
  // Commit a captured word only when BOOM completes the register read.
  capturePageWord := state === V4_Ctrl_State.STATE_RUN && ctrl_regacc_rd &&
    ctrl_header === "b0111".U && will_next
  when(capturePageWord) {
    page_ptr_in_rw := page_ptr_in_rw + 1.U
  }

  // Prefetch word zero as the command enters the ROM. On each completed
  // register write, issue the next SRAM address in the same clock cycle.
  // With a stalled BOOM, the index and the exposed word remain unchanged.
  val writingPage = state === V4_Ctrl_State.STATE_RUN &&
    state_register === V4_OP_OP.STATE_PGWT
  val consumePageWord = writingPage && ctrl_regacc_wt &&
    ctrl_header === "b1010".U && !cpu_regacc_busy
  when(state === V4_Ctrl_State.STATE_DO_OP && opcode === V4_SER_OP.SEROP_PGWT) {
    pageReadEnable := true.B
  }
  when(writingPage) {
    pageReadEnable := true.B
    pageReadAddress := (page_ptr_in_rw + consumePageWord.asUInt)(5, 0)
    when(consumePageWord) {
      page_ptr_in_rw := page_ptr_in_rw + 1.U
    }
  }
}
