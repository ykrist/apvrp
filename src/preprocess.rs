use crate::*;
use instances::dataset::apvrp::ApvrpInstance;
use tracing::trace;
/// Group active vehicles into groups based on PV-AV compatibilities.
#[tracing::instrument(skip(data), fields(?data.id))]
pub fn av_grouping(data: ApvrpInstance) -> Data {
  let mut compat_active_passive = map_with_capacity(data.n_active);

  for (&pv, avs) in &data.compat_passive_active {
    for &av in avs {
      compat_active_passive.entry(av).or_insert_with(Vec::new).push(pv);
    }
  }

  let mut grouped_by_pv_compat = map_with_capacity(data.n_active);
  for (&av, pvs) in compat_active_passive.iter_mut() {
    pvs.sort();
    grouped_by_pv_compat.entry(pvs.clone()).or_insert_with(Vec::new).push(av);
  }

  let mut av_groups = map_with_capacity(grouped_by_pv_compat.len());
  let mut compat_passive_active = map_with_capacity(data.n_passive);


  for (pvs, mut avs) in grouped_by_pv_compat {
    avs.sort();
    let avg = avs[0];
    for av in &avs[1..] {
      compat_active_passive.remove(av).expect("missing AV");
    }
    av_groups.insert(avg, avs);

    for pv in pvs {
      compat_passive_active.entry(pv).or_insert_with(Vec::new).push(avg);
    }
  }

  for avs in compat_passive_active.values_mut() {
    avs.sort()
  }
  trace!(av_groups=?&av_groups);
  Data {
    id: data.id,
    odepot: data.odepot,
    ddepot: data.ddepot,
    n_req: data.n_req,
    n_passive: data.n_passive,
    n_active: data.n_active,
    n_loc: data.n_loc,
    tmax: data.tmax,
    srv_time: data.srv_time,
    start_time: data.start_time,
    end_time: data.end_time,
    compat_req_passive: data.compat_req_passive,
    compat_passive_req: data.compat_passive_req,
    compat_passive_active: compat_passive_active,
    compat_active_passive: compat_active_passive,
    travel_cost: data.travel_cost,
    travel_time: data.travel_time,
    av_groups
  }
}

/// Removes PV-request compatibilities when a passive vehicle cannot service a request due to time constraints.
pub fn pv_req_timing_compat(data: &mut ApvrpInstance) {
  let mut compat_req_passive = map_with_capacity(data.n_req);

  for (&po, reqs) in data.compat_passive_req.iter_mut() {
    let mut new_reqs = Vec::new();

    for &rp in &*reqs {
      let rd = rp + data.n_req;
      if data.travel_time[&(data.odepot, po)] +
        data.travel_time[&(po, rp)] +
        data.srv_time[&rp] +
        data.travel_time[&(rp, rd)] +
        data.srv_time[&rd] <= data.end_time[&rd] {
        new_reqs.push(rp);
        compat_req_passive.entry(rp).or_insert_with(Vec::new).push(po);
      }
    }
    *reqs = new_reqs;
  }

  data.compat_req_passive = compat_req_passive;
}