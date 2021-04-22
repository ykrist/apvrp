use crate::*;
use instances::dataset::apvrp::ApvrpInstance;
use tracing::trace;
/// Group active vehicles into groups based on PV-AV compatibilities.
#[tracing::instrument(skip(data), fields(?data.id))]
pub fn av_grouping(data: ApvrpInstance, lss: &LocSetStarts) -> Data {
  let mut compat_active_passive : Map<_, Vec<_>> = map_with_capacity(data.n_active as usize);

  for (&pv, avs) in &data.compat_passive_active {
    let pv = pv - lss.pv_o;
    for &av in avs {
      compat_active_passive.entry(av).or_default().push(pv);
    }
  }

  let mut grouped_by_pv_compat: Map<_, Vec<_>> = map_with_capacity(data.n_active as usize);
  for (&av, pvs) in compat_active_passive.iter_mut() {
    pvs.sort();
    grouped_by_pv_compat.entry(pvs.clone()).or_default().push(av);
  }

  let mut av_groups = map_with_capacity(grouped_by_pv_compat.len());
  let mut compat_passive_active : Map<_, Vec<_>> = map_with_capacity(data.n_passive as usize);

  for (pvs, mut avs) in grouped_by_pv_compat {
    avs.sort();
    let avg = avs[0];
    for av in &avs[1..] {
      compat_active_passive.remove(av).expect("missing AV");
    }
    av_groups.insert(avg, avs);

    for pv in pvs {
      compat_passive_active.entry(pv).or_default().push(avg);
    }
  }

  for avs in compat_passive_active.values_mut() {
    avs.sort()
  }
  trace!(av_groups=?&av_groups);

  let decode_keys_loc =
    |(i, val): (&RawLoc, &Time)| { (Loc::decode(*i, lss), *val) };

  let decode_keys_locpair = |(&(i,j), &val): (&(RawLoc, RawLoc), &Time)| {
    ((Loc::decode(i, lss), Loc::decode(j, lss)), val)
  };

  let compat_req_passive : Map<_, Vec<_>> = data.compat_req_passive.iter()
    .map(|(&raw_req, raw_pvs)| (raw_req - lss.req_p, raw_pvs.iter().map(|&p| p - lss.pv_o).collect()))
    .collect();

  let compat_passive_req : Map<_, Vec<_>> = data.compat_passive_req.iter()
    .map(|(&raw_pv, raw_reqs)| (raw_pv - lss.pv_o, raw_reqs.iter().map(|&r| r - lss.req_p).collect()))
    .collect();


  Data {
    id: data.id,
    n_req: data.n_req,
    n_passive: data.n_passive,
    n_active: data.n_active,
    n_loc: data.n_loc,
    tmax: data.tmax,
    srv_time: data.srv_time.iter().map(decode_keys_loc).collect(),
    start_time: data.start_time.iter().map(decode_keys_loc).collect(),
    end_time: data.end_time.iter().map(decode_keys_loc).collect(),
    compat_req_passive,
    compat_passive_req,
    compat_passive_active,
    compat_active_passive,
    travel_cost: data.travel_cost.iter().map(decode_keys_locpair).collect(),
    travel_time: data.travel_time.iter().map(decode_keys_locpair).collect(),
    av_groups
  }
}

/// Removes PV-request compatibilities when a passive vehicle cannot service a request due to time constraints.
pub fn pv_req_timing_compat(data: &mut ApvrpInstance) {
  let mut compat_req_passive = map_with_capacity(data.n_req as usize);

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