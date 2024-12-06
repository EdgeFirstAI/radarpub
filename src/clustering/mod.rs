use std::collections::{HashMap, HashSet, VecDeque};

use dbscan::{Classification, Model};
use tracker::{ByteTrack, TrackSettings, VAALBox};
use uuid::Uuid;

mod kalman;
mod tracker;
#[derive(Debug, Clone, Default)]
pub struct Clustering {
    /// Clustering DBSCAN distance limit (euclidean distance)
    clustering_eps: f64,

    /// Clustering DBSCAN parameter scaling. Parameter order is x, y, z, speed.
    /// Set the appropriate axis to 0 to ignore that axis
    clustering_param_scale: Vec<f32>,

    /// Clustering DBSCAN point limit. Minimum 3
    clustering_point_limit: usize,

    /// Tracker
    tracker: ByteTrack,

    /// Settings for tracking
    track_settings: TrackSettings,

    /// track id to cluster id
    track_id_to_cluster_id: HashMap<Uuid, usize>,

    /// available cluster ids
    cluster_id_queue: VecDeque<usize>,

    /// max_cluster_id
    cluster_id_max: usize,
}

impl Clustering {
    pub fn new(
        clustering_eps: f64,
        clustering_param_scale: &[f32],
        clustering_point_limit: usize,
    ) -> Self {
        let mut clustering_param_scale = clustering_param_scale.to_vec();
        while clustering_param_scale.len() < 4 {
            clustering_param_scale.push(0.0);
        }
        Clustering {
            clustering_eps,
            clustering_param_scale,
            clustering_point_limit,
            tracker: ByteTrack::new(),
            track_settings: TrackSettings::default(),
            track_id_to_cluster_id: HashMap::new(),
            cluster_id_queue: VecDeque::new(),
            cluster_id_max: 0,
        }
    }

    /// Clusters radar points. Radar points should be given as a list of tuples
    /// of 4 elements [(x, y, z, speed), (x, y, z, speed), ...]
    ///
    /// Radar clusters will be returned as a list of tuples of 5 elements
    /// [(x, y, z, speed, cluster_id), (x, y, z, speed, cluster_id), ...]
    /// Points with a cluster_id = 0 are noise. Otherwise points with the same
    /// cluster_id are in the same cluster
    pub fn cluster(&mut self, targets: Vec<[f32; 4]>, timestamp: u64) -> Vec<[f32; 5]> {
        let dbscantargets: Vec<Vec<f32>> = targets
            .iter()
            .map(|t| {
                let mut v = Vec::from(t);
                for (i, val) in v.iter_mut().enumerate() {
                    *val *= self.clustering_param_scale[i];
                }
                v
            })
            .collect();
        let dbscan_clusters =
            Model::new(self.clustering_eps, self.clustering_point_limit).run(&dbscantargets);
        // do some tracking to keep cluster_ids consistent across different runs

        let mut data: Vec<_> = targets
            .iter()
            .zip(dbscan_clusters.iter())
            .map(|(target, cluster)| {
                let cluster_id = match cluster {
                    Classification::Core(i) => i + 1,
                    Classification::Edge(i) => i + 1,
                    Classification::Noise => 0,
                };
                [
                    target[0],
                    target[1],
                    target[2],
                    target[3],
                    cluster_id as f32,
                ]
            })
            .collect();

        let mut boxes = Vec::new();
        let mut clusters = HashMap::new();
        for p in data.iter() {
            let id = p[4] as usize;
            clusters.entry(id).or_insert_with(Vec::new);
            clusters.get_mut(&id).unwrap().push(*p)
        }
        for (id, cluster) in clusters {
            if id == 0 {
                continue;
            }
            if cluster.is_empty() {
                continue;
            }
            let mut xmin = 9999999.9;
            let mut xmax = -9999999.9;
            let mut ymin = 9999999.9;
            let mut ymax = -9999999.9;
            for p in cluster {
                xmin = p[0].min(xmin);
                xmax = p[0].max(xmax);
                ymin = p[1].min(ymin);
                ymax = p[1].max(ymax);
            }
            if xmax - xmin < self.clustering_eps as f32 * 2.0 {
                xmax = (xmax + xmin) / 2.0 + self.clustering_eps as f32 / 2.0;
                xmin = (xmax + xmin) / 2.0 - self.clustering_eps as f32 / 2.0;
            }
            if ymax - ymin < self.clustering_eps as f32 * 2.0 {
                ymax = (ymax + ymin) / 2.0 + self.clustering_eps as f32 / 2.0;
                ymin = (ymax + ymin) / 2.0 - self.clustering_eps as f32 / 2.0;
            }
            boxes.push(VAALBox {
                xmin,
                ymin,
                xmax,
                ymax,
                score: 1.0,
                label: id as i32,
            });
            // let mut xsum = 0.0;
            // let mut ysum = 0.0;
            // for p in cluster.iter() {
            //     xsum += p[0];
            //     ysum += p[1];
            // }

            // boxes.push(VAALBox {
            //     xmin: xsum / cluster.len() as f32 - 0.3,
            //     ymin: ysum / cluster.len() as f32 - 0.3,
            //     xmax: xsum / cluster.len() as f32 + 0.3,
            //     ymax: ysum / cluster.len() as f32 + 0.3,
            //     score: 1.0,
            //     label: id as i32,
            // });
        }
        let trackinfo = self
            .tracker
            .update(&self.track_settings, &mut boxes, timestamp);
        let mut old_to_new = HashMap::new();
        for (ind, info) in trackinfo.into_iter().enumerate() {
            if info.is_none() {
                continue;
            }
            let info = info.unwrap();
            let old_cluster_id = boxes[ind].label;
            let new_cluster_id = match self.track_id_to_cluster_id.get(&info.uuid) {
                None => {
                    let new_id = self.get_new_cluster_id();
                    self.track_id_to_cluster_id.insert(info.uuid, new_id);
                    new_id
                }
                Some(v) => *v,
            };
            // let new_cluster_id = (info.uuid.as_u128() % 32) as i32;
            old_to_new.insert(old_cluster_id, new_cluster_id);
        }
        for d in data.iter_mut() {
            if d[4] == 0.0 {
                continue;
            }
            d[4] = old_to_new[&(d[4] as i32)] as f32;
        }

        let mut remove_track: HashSet<_> = self.track_id_to_cluster_id.keys().cloned().collect();
        for tracklet in self.tracker.get_tracklets() {
            let _ = remove_track.remove(&tracklet.id);
        }
        for track_id in remove_track {
            let cluster_id = self.track_id_to_cluster_id.remove(&track_id);
            if let Some(v) = cluster_id {
                self.cluster_id_queue.push_back(v);
            }
        }
        data
    }

    fn get_new_cluster_id(&mut self) -> usize {
        if self.cluster_id_queue.is_empty() {
            self.cluster_id_max += 1;
            self.cluster_id_max
        } else {
            self.cluster_id_queue.pop_front().unwrap()
        }
    }

    pub fn get_tracklets(&mut self) -> Vec<Vec<f32>> {
        let tracklets = self.tracker.get_tracklets();
        let mut ret = Vec::new();
        for t in tracklets {
            let vaalbox = t.get_predicted_location();
            ret.push(vec![vaalbox.xmin, vaalbox.ymin, vaalbox.xmax, vaalbox.ymax]);
        }
        ret
    }
}
