use crate::logging::*;
use crate::*;

use smallvec::SmallVec;
use solution::PvRoute;
use std::cmp::max;

pub type RouteId = usize;

#[derive(Debug, Clone)]
pub struct PvRoutes {
  pub by_id: Vec<PvRoute>,
  pub by_pvtask: Map<PvTask, Vec<RouteId>>,
}

impl PvRoutes {
  pub fn new(sets: &Sets, data: &Data, tasks: &Tasks) -> Self {
    let mut routes = Vec::new();
    let mut by_pvtask: Map<PvTask, Vec<usize>> = map_with_capacity(tasks.pvtask_to_task.len());

    for pv in sets.pvs() {
      let _s = error_span!("pvcg", pv = pv.0).entered();
      let pv_routes = RouteGenerator::new(data, tasks, pv).generate_routes();
      info!(count = pv_routes.len(), "generated routes for PV");
      for r in pv_routes {
        let id = routes.len();
        for &t in r.iter() {
          by_pvtask.entry(t).or_default().push(id);
        }
        routes.push(r);
      }
    }

    PvRoutes {
      by_id: routes,
      by_pvtask,
    }
  }
}

type RequestList = SmallVec<[Req; constants::NUM_REQ_UB]>;

struct RouteGenerator<'a> {
  // The dest depot for this passive vehicle
  ddepot: Loc,
  // The lastest time the PV can arrive at the dest depot
  ddepot_deadline: Time,
  pv: Pv,
  all_requests: &'a [Req],
  routes: Vec<PvRoute>,
  data: &'a Data,
  tasks: &'a Tasks,
}

impl<'a> RouteGenerator<'a> {
  pub fn new(data: &'a Data, tasks: &'a Tasks, pv: Pv) -> Self {
    let ddepot = Loc::Pd(pv);
    Self {
      data,
      tasks,
      pv,
      ddepot,
      ddepot_deadline: data.tmax - data.travel_time[&(ddepot, Loc::Ad)],
      all_requests: data.compat_passive_req[&pv].as_slice(),
      routes: Vec::new(),
    }
  }

  #[instrument(level = "trace", skip(self))]
  fn add_new_route(&mut self, reqlist: &RequestList) {
    let k = reqlist.len();
    if k == 0 {
      self.routes.push(PvRoute::new(vec![
        self.tasks.by_index_pv[&IdxPvTask::Direct(self.pv)],
      ]));
      return;
    }
    let n = k * 2 + 1;
    let mut route = Vec::with_capacity(n);
    route.push(self.tasks.by_index_pv[&IdxPvTask::Start(self.pv, reqlist[0])]);

    for (&r1, &r2) in reqlist.iter().tuple_windows() {
      route.push(self.tasks.by_index_pv[&IdxPvTask::Request(self.pv, r1)]);
      route.push(self.tasks.by_index_pv[&IdxPvTask::Transfer(self.pv, r1, r2)]);
    }

    route.push(self.tasks.by_index_pv[&IdxPvTask::Request(self.pv, reqlist[k - 1])]);
    route.push(self.tasks.by_index_pv[&IdxPvTask::End(self.pv, reqlist[k - 1])]);

    #[cfg(debug_assertions)]
    {
      if !schedule::check_pv_route(self.data, &route) {
        error!(?route, "illegal route");
        panic!("bugalug")
      }
    }
    self.routes.push(PvRoute::new(route));
  }

  /// Resources:
  /// - The location we departed from
  /// - Time we depart the delivery loc of the last request
  /// - Set of requests visited so far
  #[instrument(level = "trace", skip(self, requests))]
  fn extend_routes(&mut self, loc: Loc, time: Time, requests: RequestList) {
    // extend to depot
    if time + self.data.travel_time[&(loc, self.ddepot)] > self.ddepot_deadline {
      return;
    }

    self.add_new_route(&requests);

    for next_req in self.all_requests {
      if requests.contains(next_req) {
        continue;
      }
      let next_req = *next_req;
      let next_pickup = Loc::ReqP(next_req);
      let t = max(
        time + self.data.travel_time[&(loc, next_pickup)] + self.data.srv_time[&next_pickup],
        self.data.start_time[&next_pickup],
      );
      let next_delivery = Loc::ReqD(next_req);
      let t = t
        + self.data.travel_time[&(next_pickup, next_delivery)]
        + self.data.srv_time[&next_delivery];
      if t > self.data.end_time[&next_delivery] {
        continue;
      }
      let mut new_requests = requests.clone();
      new_requests.push(next_req);
      self.extend_routes(Loc::ReqD(next_req), t, new_requests);
    }
  }

  pub fn generate_routes(mut self) -> Vec<PvRoute> {
    trace!(?self.all_requests);
    for &r in self.all_requests {
      let p = Loc::ReqP(r);
      let d = Loc::ReqD(r);
      let t = self.data.pv_req_start_time[&(self.pv, r)]
        + self.data.travel_time[&(p, d)]
        + self.data.srv_time[&d];
      self.extend_routes(d, t, smallvec::smallvec![r]);
    }
    self.routes
  }
}
