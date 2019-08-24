#ifndef EXE_XIL_COMMON_MODIFIED_DATA_GPS_H_
#define EXE_XIL_COMMON_MODIFIED_DATA_GPS_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"

extern "C" void record_gps()
{
    printf("HELLO");
    Trick::DRAscii *drg = new Trick::DRAscii("ngps");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.05);
    drg->set_single_prec_only(false);
    drg->add_variable("dyn.V1.DM->_FSPB[0]");
    drg->add_variable("dyn.V1.DM->_FSPB[1]");
    drg->add_variable("dyn.V1.DM->_FSPB[2]");
    drg->add_variable("dyn.V1.DM->_ABIB[0]");
    drg->add_variable("dyn.V1.DM->_ABIB[1]");
    drg->add_variable("dyn.V1.DM->_ABIB[2]");
    drg->add_variable("dyn.V1.DM->_FMAB[0]");
    drg->add_variable("dyn.V1.DM->_FMAB[1]");
    drg->add_variable("dyn.V1.DM->_FMAB[2]");
    drg->add_variable("dyn.V1.DM->_FAPB[0]");
    drg->add_variable("dyn.V1.DM->_FAPB[1]");
    drg->add_variable("dyn.V1.DM->_FAPB[2]");
    drg->add_variable("dyn.V1.DM->_WBIBD[0]");
    drg->add_variable("dyn.V1.DM->_WBIBD[1]");
    drg->add_variable("dyn.V1.DM->_WBIBD[2]");
    drg->add_variable("dyn.V1.DM->_NEXT_ACC[0]");
    drg->add_variable("dyn.V1.DM->_NEXT_ACC[1]");
    drg->add_variable("dyn.V1.DM->_NEXT_ACC[2]");
    drg->add_variable("dyn.V1.Prop->thrust");
    drg->add_variable("dyn.V1.Prop->vmass");
    drg->add_variable("dyn.V1.DM->phibdx");
    drg->add_variable("dyn.V1.DM->thtbdx");
    drg->add_variable("dyn.V1.DM->psibdx");
    drg->add_variable("dyn.V1.DM->alphax");
    drg->add_variable("dyn.V1.DM->alppx");
    drg->add_variable("dyn.V1.DM->lonx");
    drg->add_variable("dyn.V1.DM->latx");
    drg->add_variable("dyn.V1.DM->alt");
    drg->add_variable("dyn.V1.Env->vmach");
    drg->add_variable("dyn.V1.DM->_FAPB[0]");
    drg->add_variable("dyn.V1.DM->_FAPB[1]");
    drg->add_variable("dyn.V1.DM->_FAPB[2]");
    drg->add_variable("dyn.V1.DM->_FMAB[0]");
    drg->add_variable("dyn.V1.DM->_FMAB[1]");
    drg->add_variable("dyn.V1.DM->_FMAB[2]");
    drg->add_variable("dyn.V1.DM->_WBIB[0]");
    drg->add_variable("dyn.V1.DM->_WBIB[1]");
    drg->add_variable("dyn.V1.DM->_WBIB[2]");
    drg->add_variable("dyn.V1.Env->pdynmc");
    drg->add_variable("dyn.V1.DM->_f[0]");
    drg->add_variable("dyn.V1.DM->_f[1]");
    drg->add_variable("dyn.V1.DM->_f[2]");
    drg->add_variable("dyn.V1.DM->_f[3]");
    drg->add_variable("dyn.V1.DM->_f[4]");
    drg->add_variable("dyn.V1.DM->_f[5]");
    drg->add_variable("dyn.V1.DM->_p_b1_ga[0]");
    drg->add_variable("dyn.V1.DM->_p_b1_ga[1]");
    drg->add_variable("dyn.V1.DM->_p_b1_ga[2]");
    drg->add_variable("dyn.V1.DM->_p_b1_be[0]");
    drg->add_variable("dyn.V1.DM->_p_b1_be[1]");
    drg->add_variable("dyn.V1.DM->_p_b1_be[2]");
    drg->add_variable("dyn.V1.DM->_Q_Aero[0]");
    drg->add_variable("dyn.V1.DM->_Q_Aero[1]");
    drg->add_variable("dyn.V1.DM->_Q_Aero[2]");
    drg->add_variable("dyn.V1.DM->_Q_Aero[3]");
    drg->add_variable("dyn.V1.DM->_Q_Aero[4]");
    drg->add_variable("dyn.V1.DM->_Q_Aero[5]");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // EXE_XIL_COMMON_MODIFIED_DATA_GPS_H_
