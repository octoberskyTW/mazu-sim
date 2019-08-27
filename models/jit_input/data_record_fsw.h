#ifndef DATA_RECORD_FSW_H
#define DATA_RECORD_FSW_H

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_gps_slave()
{
    Trick::DRAscii *drg = new Trick::DRAscii("ngps_slave");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.005);
    drg->set_single_prec_only(false);
    drg->add_variable("fc.ins._VBIIC[0]");
    drg->add_variable("fc.ins._VBIIC[1]");
    drg->add_variable("fc.ins._VBIIC[2]");
    drg->add_variable("fc.ins._SBIIC[0]");
    drg->add_variable("fc.ins._SBIIC[1]");
    drg->add_variable("fc.ins._SBIIC[2]");
    drg->add_variable("fc.ins.phibdcx");
    drg->add_variable("fc.ins.thtbdcx");
    drg->add_variable("fc.ins.psibdcx");
    drg->add_variable("fc.control.vmach");
    drg->add_variable("fc.ins.alppcx");
    drg->add_variable("fc.control.pdynmc");
    drg->add_variable("fc.rcs_fc.e_roll");
    drg->add_variable("fc.rcs_fc.e_pitch");
    drg->add_variable("fc.rcs_fc.e_yaw");
    drg->add_variable("fc.control.delecx");
    drg->add_variable("fc.control.delrcx");
    drg->add_variable("fc.control.vmass");
    drg->add_variable("fc.control.fmasse");
    drg->add_variable("fc.ins.alphacx");
    drg->add_variable("fc.ins.loncx");
    drg->add_variable("fc.ins.latcx");
    drg->add_variable("fc.ins.altc");

    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // DATA_RECORD_FSW_H
