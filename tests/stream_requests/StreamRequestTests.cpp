/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: StreamRequestTests.cpp                           */
/*    DATE: November 4, 2026                                */
/************************************************************/
#include "ObservationRequests.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using mavmoos::runtime::ObservationRequests;
using mavmoos::runtime::ObservationStream;
using State=ObservationRequests::State;
namespace {
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
auto one=std::vector<ObservationStream>{{33,50000}};
const double not_a_number=std::numeric_limits<double>::quiet_NaN();
void trace(const char* label,const ObservationRequests& q)
{std::cout<<"{\"type\":\"state\",\"label\":\""<<label<<"\",\"value\":"<<q.status()<<"}\n";}
ObservationRequests::Attempt take(ObservationRequests& q,double now)
{auto a=q.take(now);check(a.has_value(),"expected a ready attempt");return *a;}
}

int main()
{
  using Case=std::pair<const char*,std::function<void()>>;
  std::vector<Case> cases={
    {"idle_requires_admission",[]{ObservationRequests q;check(!q.started()&&!q.ready(100)&&!q.take(100),"idle emitted work");trace("idle",q);}},
    {"empty_plan_is_once_only",[]{ObservationRequests q;check(q.start({},100)&&q.started()&&q.size()==0&&!q.ready(100),"empty plan invalid");check(!q.start(one,101),"empty plan admitted twice");trace("empty",q);}},
    {"duplicate_minimum_and_stable_order",[]{ObservationRequests q;check(q.start({{33,100000},{24,500000},{33,50000}},100),"admission failed");check(q.size()==2&&q.demand(0).message_id==33&&q.demand(0).interval_us==50000&&q.demand(1).message_id==24,"dedup/order/minimum wrong");trace("dedup",q);}},
    {"canonical_capacity_is_atomic",[]{ObservationRequests q;std::vector<ObservationStream> input;for(unsigned i=0;i<33;++i)input.push_back({i,10000});check(!q.start(input,100)&&q.started()&&q.size()==0&&!q.ready(100),"partial overcapacity plan escaped");trace("capacity",q);}},
    {"raw_capacity_bounds_duplicate_work",[]{ObservationRequests q;check(!q.start(std::vector<ObservationStream>(1025,{33,10000}),100)&&q.size()==0,"oversized input admitted");trace("raw_capacity",q);}},
    {"raw_capacity_boundary_is_valid",[]{ObservationRequests q;check(q.start(std::vector<ObservationStream>(1024,{33,10000}),100)&&q.size()==1,"valid bounded duplicate input rejected");}},
    {"invalid_demands_do_not_partially_admit",[]{for(auto bad:std::vector<ObservationStream>{{0x1000000,10000},{33,0},{33,-1},{33,9999},{33,60000004},{33,float(not_a_number)}}){ObservationRequests q;check(!q.start({{24,10000},bad},100)&&q.size()==0&&!q.take(100),"invalid demand partly admitted");}}},
    {"valid_wire_boundaries",[]{ObservationRequests q;check(q.start({{0,10000},{0xffffff,60000000}},100)&&q.size()==2,"valid descriptor bounds rejected");}},
    {"start_cannot_renew_lifetime",[]{ObservationRequests q;check(q.start(one,100),"start");const auto expiry=q.demand(0).expires;check(!q.start(one,101)&&q.demand(0).expires==expiry&&expiry==110,"restart renewed bound");check(!q.ready(110)&&q.demand(0).state==State::Expired,"equality not expired");trace("fixed_bound",q);}},
    {"start_rejection_still_advances_expiry",[]{ObservationRequests q;q.start(one,100);check(!q.start(one,111)&&q.demand(0).state==State::Expired,"start reused stale state");}},
    {"single_inflight_and_exact_copy",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);check(q.inFlight()&&!q.ready(100.1)&&!q.take(100.1)&&q.eligible(a,100.1),"inflight escaped or lost");trace("inflight",q);}},
    {"copied_witness_cannot_be_tampered",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);for(unsigned field=0;field<5;++field){auto bad=a;if(field==0)++bad.epoch;if(field==1)++bad.message_id;if(field==2)bad.interval_us+=1;if(field==3)++bad.attempt;if(field==4)bad.expires+=1;check(!q.eligible(bad,100.1),"tampered witness eligible");}check(q.eligible(a,100.1),"valid witness lost");}},
    {"local_rejection_retries_and_stops_on_acceptance",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);auto r=q.complete(a,100,false);check(r.matched_current&&!r.local_accepted&&r.terminal_state=="RETRY_PENDING","rejection result wrong");check(!q.ready(100.199999),"retry too early");auto b=take(q,100.2);check(b.attempt==2&&b.expires==a.expires,"retry renewed bound");auto accepted=q.complete(b,100.2,true);check(accepted.matched_current&&accepted.local_accepted&&!q.ready(101)&&q.demand(0).state==State::LocalAccepted,"accepted work retried");trace("recovered_local_loss",q);}},
    {"retry_backoff_does_not_block_other_demand",[]{ObservationRequests q;q.start({{33,50000},{24,500000}},100);auto a=take(q,100);q.complete(a,100,false);auto b=take(q,100.1);check(b.message_id==24,"failed head blocked another demand");q.complete(b,100.1,true);auto retry=take(q,100.201);check(retry.message_id==33&&retry.attempt==2,"failed demand not revisited fairly");}},
    {"global_spacing_uses_post_return_time",[]{ObservationRequests q;q.start({{33,50000},{24,500000}},100);auto a=take(q,100);q.complete(a,100.04,true);check(!q.ready(100.139999),"global spacing too short");check(take(q,100.141).message_id==24,"paced next demand missing");}},
    {"one_success_never_retried",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);q.complete(a,100,true);for(double now:{100.1,101.,109.999,110.,200.})check(!q.take(now),"successful request resent");check(q.demand(0).attempts==1&&q.demand(0).state==State::LocalAccepted,"accepted state changed with age");}},
    {"attempt_budget_is_fixed",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);q.complete(a,100,false);auto b=take(q,100.2);q.complete(b,100.2,false);auto c=take(q,100.401);auto r=q.complete(c,100.401,false);check(c.attempt==3&&r.terminal_state=="EXHAUSTED"&&!q.ready(101)&&q.demand(0).attempts==3,"rejection budget unbounded");trace("exhausted",q);}},
    {"whole_plan_capacity_and_attempt_work_bound",[]{ObservationRequests q;std::vector<ObservationStream> v;for(unsigned i=0;i<32;++i)v.push_back({i,10000});q.start(v,0);unsigned calls=0;for(unsigned step=0;step<1000;++step){double now=step*.011;if(auto a=q.take(now)){++calls;q.complete(*a,now,false);}}check(calls<=96,"bounded plan exceeded96 calls");for(unsigned i=0;i<32;++i)check(q.demand(i).attempts<=3&&q.demand(i).expires==10,"work/deadline limit changed");check(!q.ready(20),"plan revived");trace("bounded_plan",q);}},
    {"expiry_fences_dequeued_work",[]{ObservationRequests q;q.start(one,100);auto a=take(q,109.9);check(!q.eligible(a,110),"expiry equality remained eligible");check(!q.take(110),"expired request retried");auto r=q.complete(a,110,false);check(r.matched_current&&r.late&&r.terminal_state=="EXPIRED","expired receipt wrong");trace("expired_inflight",q);}},
    {"accepted_receipt_after_deadline_is_not_retried_or_hidden",[]{ObservationRequests q;q.start(one,100);auto a=take(q,109.99);auto r=q.complete(a,110.01,true);check(r.matched_current&&r.local_accepted&&r.late&&r.terminal_state=="LOCAL_ACCEPTED","late actual acceptance hidden");check(q.demand(0).accepted_receipt_late&&q.demand(0).deadline_elapsed&&!q.ready(111),"late acceptance retried");trace("accepted_late_receipt",q);}},
    {"expired_waiting_work_never_consumes_attempt",[]{ObservationRequests q;q.start(one,100);check(!q.take(110)&&q.demand(0).attempts==0&&q.demand(0).state==State::Expired,"expired work attempted");}},
    {"reset_fences_queued_and_dequeued_work",[]{ObservationRequests q;q.start({{33,50000},{24,500000}},100);auto old=take(q,100);q.reset(100.1,"connection_reset");check(!q.started()&&!q.eligible(old,100.1)&&q.size()==0,"reset preserved old work");trace("reset",q);}},
    {"new_plan_cannot_rebind_old_witness",[]{ObservationRequests q;q.start(one,100);auto old=take(q,100);q.reset(100.1,"reconfigure");q.start(one,100.1);check(!q.eligible(old,100.2),"old witness crossed fresh admission");auto current=take(q,100.2);check(current.epoch!=old.epoch&&current.expires==110.1,"new plan identity wrong");}},
    {"old_actual_acceptance_remains_event_only",[]{ObservationRequests q;q.start(one,100);auto old=take(q,100);q.reset(100.1,"reset_during_write");q.start({{24,500000}},100.1);auto r=q.complete(old,100.2,true);check(!r.matched_current&&r.local_accepted&&r.clock_valid&&q.demand(0).state==State::Queued&&q.demand(0).attempts==0,"old receipt mutated fresh plan");check(take(q,100.2).message_id==24,"old receipt blocked new work");trace("unmatched_actual_acceptance",q);}},
    {"duplicate_receipt_does_not_repeat_effect",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);q.complete(a,100,true);auto again=q.complete(a,100.1,true);check(!again.matched_current&&again.local_accepted&&q.demand(0).attempts==1&&!q.ready(100.2),"duplicate receipt changed plan");}},
    {"forged_receipt_cannot_release_current_attempt",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100);auto fake=a;++fake.message_id;auto r=q.complete(fake,100.1,true);check(!r.matched_current&&q.inFlight()&&q.eligible(a,100.1),"forged receipt released live work");}},
    {"backwards_clock_latches_and_retires_work",[]{ObservationRequests q;q.start(one,100);auto a=take(q,100.1);check(!q.eligible(a,100.09)&&q.clockFault(),"backwards clock accepted");check(q.demand(0).state==State::Cancelled&&!q.ready(101)&&!q.start(one,101),"clock fault recovered implicitly");auto r=q.complete(a,101,true);check(!r.matched_current&&r.local_accepted&&!r.clock_valid,"clock-fault receipt overclaimed");trace("clock_fault",q);}},
    {"nonfinite_clocks_fail_closed",[]{for(double bad:{not_a_number,std::numeric_limits<double>::infinity(),-1.}){ObservationRequests q;q.start(one,100);auto a=take(q,100);check(!q.eligible(a,bad)&&q.clockFault()&&!q.take(101),"invalid clock allowed work");}}},
    {"explicit_valid_reset_recovers_clock_fault",[]{ObservationRequests q;q.start(one,100);q.ready(99);q.reset(99,"bad_rebase");check(q.clockFault()&&!q.start(one,100),"reset rebased backwards");q.reset(100,"clock_recovered");check(!q.clockFault()&&q.start(one,100)&&q.ready(100),"valid explicit reset could not recover");}},
    {"unrepresentable_deadline_has_no_work",[]{ObservationRequests q;check(!q.start(one,std::numeric_limits<double>::max())&&q.size()==0,"unrepresentable deadline accepted");trace("unrepresentable",q);}},
    {"unrepresentable_pacing_has_no_work",[]{ObservationRequests q;check(!q.start(one,1e16)&&q.size()==0&&!q.take(1e16),"unrepresentable pacing accepted");trace("unrepresentable_pacing",q);}},
    {"diagnostic_reason_is_bounded_and_json_safe",[]{ObservationRequests q;q.reset(100,std::string(300,'x')+"\n\"");check(q.status().size()<1000,"unbounded reason");q.reset(100,"quoted\" slash\\ newline\n");trace("escaped",q);}},
    {"demand_accessor_is_bounded",[]{ObservationRequests q;q.start(one,100);bool rejected=false;try{q.demand(1);}catch(const std::out_of_range&){rejected=true;}check(rejected,"out of bounds demand access");}}
  };
  try {
    for(const auto& test:cases){test.second();std::cout<<"{\"type\":\"pass\",\"case\":\""<<test.first<<"\"}\n";}
    std::cout<<"{\"type\":\"complete\",\"groups\":"<<cases.size()<<",\"io\":\"none\"}\n";return 0;
  } catch(const std::exception& e) {std::cerr<<"stream request failure: "<<e.what()<<'\n';return 1;}
}
