use crate::*;

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