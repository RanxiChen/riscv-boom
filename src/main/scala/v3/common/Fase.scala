package boom.v3.common

import chisel3._
import org.chipsalliance.cde.config.Field

/** Opt-in; the native board configuration keeps its original CPU and UART. */
case object BoomFaseKey extends Field[Boolean](false)
case object BoomFaseMicrocodeKey extends Field[Boolean](false)
class FaseUartPort extends Bundle {
  val rxd = Input(Bool())
  val txd = Output(Bool())
  val state = Output(UInt(8.W))
}
