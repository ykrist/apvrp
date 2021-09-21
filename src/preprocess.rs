use crate::*;
use instances::dataset::apvrp::ApvrpInstance;
use tracing::{info, info_span, trace};

/// For each Passive Vehicle-Request pair, computes the earliest time we can *leave* the pickup of the request.
#[tracing::instrument(level = "trace", skip(compat_req_passive, travel_time, srv_time, start_time))]
fn earliest_departures(
  compat_req_passive: &Map<Req, Vec<Pv>>,
  travel_time: &Map<(Loc, Loc), Time>,
  srv_time: &Map<Loc, Time>,
  start_time: &Map<Loc, Time>,
) -> Map<(Pv, Req), Time> {
  compat_req_passive.iter()
    .flat_map(|(&r, pvs)| {
      pvs.iter()
        .map(move |&p| {
          let rp = Loc::ReqP(r);
          let po = Loc::Po(p);
          trace!(?rp, ?po);
          let t = std::cmp::max(
            start_time[&rp],
            travel_time[&(Loc::Ao, po)] + travel_time[&(po, rp)] + srv_time[&rp],
          );
          ((p, r), t)
        })
    })
    .collect()
}

/// For each Passive Vehicle-Request pair, computes the arrive time we can *arrive* at the delivery of the request.
#[tracing::instrument(level = "trace", skip(compat_req_passive, travel_time, srv_time, end_time))]
fn latest_arrivals(
  compat_req_passive: &Map<Req, Vec<Pv>>,
  travel_time: &Map<(Loc, Loc), Time>,
  srv_time: &Map<Loc, Time>,
  end_time: &Map<Loc, Time>,
  tmax: Time,
) -> Map<(Pv, Req), Time> {
  compat_req_passive.iter()
    .flat_map(|(&r, pvs)| {
      pvs.iter()
        .map(move |&p| {
          let rd = Loc::ReqD(r);
          let pd = Loc::Pd(p);
          trace!(?rd, ?pd);
          let t = std::cmp::max(
            end_time[&rd],
            tmax - travel_time[&(rd, pd)] -  travel_time[&(pd, Loc::Ad)]
          ) - srv_time[&rd];
          ((p, r), t)
        })
    })
    .collect()
} //  data.end_time[&rd] - data.srv_time[&rd]

fn decode_loc_keys<V: Copy>(lss: &LocSetStarts, map: &Map<RawLoc, V>) -> Map<Loc, V> {
  map.iter()
    .map(|(key, val)| (lss.decode(*key), *val))
    .collect()
}

fn decode_locpair_keys<V: Copy>(lss: &LocSetStarts, map: &Map<(RawLoc, RawLoc), V>) -> Map<(Loc, Loc), V> {
  map.iter()
    .map(|(&(i,j), &val)| ((lss.decode(i),lss.decode(j)), val))
    .collect()
}

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

  let compat_req_passive : Map<_, Vec<_>> = data.compat_req_passive.iter()
    .map(|(&raw_req, raw_pvs)| (raw_req - lss.req_p, raw_pvs.iter().map(|&p| p - lss.pv_o).collect()))
    .collect();

  let compat_passive_req : Map<_, Vec<_>> = data.compat_passive_req.iter()
    .map(|(&raw_pv, raw_reqs)| (raw_pv - lss.pv_o, raw_reqs.iter().map(|&r| r - lss.req_p).collect()))
    .collect();

  let travel_time = decode_locpair_keys(&lss, &data.travel_time);
  let srv_time = decode_loc_keys(&lss, &data.srv_time);
  let start_time = decode_loc_keys(&lss, &data.start_time);

  let pv_req_start_time = earliest_departures(
    &compat_req_passive,
    &travel_time,
    &srv_time,
    &start_time
  );

  Data {
    id: data.id,
    n_req: data.n_req,
    n_passive: data.n_passive,
    n_active: data.n_active,
    n_loc: data.n_loc,
    tmax: data.tmax,
    srv_time,
    start_time,
    end_time: decode_loc_keys(&lss, &data.end_time),
    compat_req_passive,
    compat_passive_req,
    compat_passive_active,
    compat_active_passive,
    pv_req_start_time,
    travel_cost: decode_locpair_keys(&lss, &data.travel_cost),
    travel_time,
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

pub fn full_pipeline(mut data: ApvrpInstance) -> Data {
  let _span = info_span!("preprocess", id=%data.id).entered();
  info!("loaded instance");
  preprocess::pv_req_timing_compat(&mut data);
  let lss = LocSetStarts::new(data.n_passive, data.n_req);
  let data = preprocess::av_grouping(data, &lss);
  info!(num_av_groups = data.av_groups.len(), num_av = data.n_active, "preprocessing finished");
  data
}