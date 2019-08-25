#ifndef __DATA_RECORD_GOLDEN_H_
#define __DATA_RECORD_GOLDEN_H_

#include "trick/DRAscii.hh"
#include "trick/DataRecordGroup.hh"
#include "trick/data_record_proto.h"
extern Rocket_SimObject dyn;
extern "C" void record_golden()
{
    Trick::DRAscii *drg = new Trick::DRAscii("rocket_csv");
    drg->set_freq(Trick::DR_Always);
    drg->set_cycle(0.1);
    drg->set_single_prec_only(false);
    drg->add_variable("dyn.V1.DM->lonx");
    drg->add_variable("dyn.V1.DM->latx");
    drg->add_variable("dyn.V1.DM->alt");
    drg->add_variable("dyn.V1.DM->_dvbi");
    drg->add_variable("dyn.V1.DM->_SBII[0]");
    drg->add_variable("dyn.V1.DM->_SBII[1]");
    drg->add_variable("dyn.V1.DM->_SBII[2]");
    drg->add_variable("dyn.V1.DM->_VBII[0]");
    drg->add_variable("dyn.V1.DM->_VBII[1]");
    drg->add_variable("dyn.V1.DM->_VBII[2]");
    drg->add_variable("dyn.V1.DM->thtbdx");
    add_data_record_group(drg, Trick::DR_Buffer);
    drg->enable();
}

#endif  // __DATA_RECORD_GOLDEN_H_
