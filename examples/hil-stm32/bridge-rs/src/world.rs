//! 假雷射與碰撞用的世界(world.json):矩形房間 + 軸對齊方塊。
//! 射線投射與碰撞判斷在 plant/world.py 有同一份(fake_plant.py 用);地圖(tools/gen_map.py)也從同一份 JSON 產。
use std::fs;
use std::io;

#[derive(Debug, Clone, Copy)]
pub struct Box2 {
    pub cx: f64,
    pub cy: f64,
    pub w: f64,
    pub h: f64,
}

#[derive(Debug, Clone)]
pub struct World {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
    pub boxes: Vec<Box2>,
    pub robot_radius: f64,
    pub beams: usize,
    pub range_min: f64,
    pub range_max: f64,
    pub period_ms: u64,
    pub goal: (f64, f64, f64),
}

fn num(text: &str, key: &str) -> io::Result<f64> {
    let pat = format!("\"{}\"", key);
    let i = text.find(&pat).ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, format!("world.json 缺 {key}")))?;
    let rest = &text[i + pat.len()..];
    let rest = rest.trim_start().strip_prefix(':').unwrap_or(rest).trim_start();
    let end = rest.find(|c: char| c == ',' || c == '}' || c == '\n').unwrap_or(rest.len());
    rest[..end].trim().parse::<f64>().map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("{key}: {e}")))
}

impl World {
    pub fn load(path: &str) -> io::Result<World> {
        let t = fs::read_to_string(path)?;
        let sec = |name: &str| -> io::Result<&str> {
            let i = t.find(&format!("\"{name}\"")).ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, format!("world.json 缺 {name}")))?;
            Ok(&t[i..])
        };
        let room = sec("room")?;
        let laser = sec("laser")?;
        let goal = sec("goal")?;
        let mut boxes = Vec::new();
        if let Some(i) = t.find("\"boxes\"") {
            let rest = &t[i..];
            let end = rest.find(']').unwrap_or(rest.len());
            for chunk in rest[..end].split('}') {
                if chunk.contains("\"cx\"") {
                    boxes.push(Box2 { cx: num(chunk, "cx")?, cy: num(chunk, "cy")?, w: num(chunk, "w")?, h: num(chunk, "h")? });
                }
            }
        }
        Ok(World {
            x_min: num(room, "x_min")?, x_max: num(room, "x_max")?, y_min: num(room, "y_min")?, y_max: num(room, "y_max")?,
            boxes,
            robot_radius: num(&t, "robot_radius_m")?,
            beams: num(laser, "beams")? as usize,
            range_min: num(laser, "range_min_m")?,
            range_max: num(laser, "range_max_m")?,
            period_ms: num(laser, "period_ms")? as u64,
            goal: (num(goal, "x")?, num(goal, "y")?, num(goal, "yaw")?),
        })
    }

    /// 所有牆與方塊的邊,當線段 (x1,y1)-(x2,y2)
    fn segments(&self) -> Vec<(f64, f64, f64, f64)> {
        let mut s = vec![
            (self.x_min, self.y_min, self.x_max, self.y_min),
            (self.x_max, self.y_min, self.x_max, self.y_max),
            (self.x_max, self.y_max, self.x_min, self.y_max),
            (self.x_min, self.y_max, self.x_min, self.y_min),
        ];
        for b in &self.boxes {
            let (x0, y0, x1, y1) = (b.cx - b.w / 2.0, b.cy - b.h / 2.0, b.cx + b.w / 2.0, b.cy + b.h / 2.0);
            s.push((x0, y0, x1, y0)); s.push((x1, y0, x1, y1)); s.push((x1, y1, x0, y1)); s.push((x0, y1, x0, y0));
        }
        s
    }

    /// 從 (x, y, th) 射 beams 束(第 0 束朝 th − π,逆時針,與 LaserScan 的 angle_min = −π 對應),回距離(m);沒打到 = range_max
    pub fn scan(&self, x: f64, y: f64, th: f64) -> Vec<f32> {
        let segs = self.segments();
        let mut out = Vec::with_capacity(self.beams);
        for i in 0..self.beams {
            let a = th - std::f64::consts::PI + 2.0 * std::f64::consts::PI * i as f64 / self.beams as f64;
            let (dx, dy) = (a.cos(), a.sin());
            let mut best = self.range_max;
            for &(x1, y1, x2, y2) in &segs {
                // 射線 p + t·d 與線段 q + u·(r) 的交點
                let (rx, ry) = (x2 - x1, y2 - y1);
                let den = dx * ry - dy * rx;
                if den.abs() < 1e-12 { continue; }
                let t = ((x1 - x) * ry - (y1 - y) * rx) / den;
                let u = ((x1 - x) * dy - (y1 - y) * dx) / den;
                if t >= 0.0 && (0.0..=1.0).contains(&u) && t < best { best = t; }
            }
            out.push(if best < self.range_min { self.range_min } else { best } as f32);
        }
        out
    }

    /// 正前方單束射線的距離(m):近距離安全區的感測器(CAN 0x301,38 篇 §1.9)。
    /// 與 scan() 同一組線段、同一個交點算法,差別只有一束、方向就是車頭
    pub fn range_ahead(&self, x: f64, y: f64, th: f64) -> f64 {
        let (dx, dy) = (th.cos(), th.sin());
        let mut best = self.range_max;
        for &(x1, y1, x2, y2) in &self.segments() {
            let (rx, ry) = (x2 - x1, y2 - y1);
            let den = dx * ry - dy * rx;
            if den.abs() < 1e-12 { continue; }
            let t = ((x1 - x) * ry - (y1 - y) * rx) / den;
            let u = ((x1 - x) * dy - (y1 - y) * dx) / den;
            if t >= 0.0 && (0.0..=1.0).contains(&u) && t < best { best = t; }
        }
        best
    }

    /// 半徑 robot_radius 的圓與任一方塊或牆相交
    pub fn collides(&self, x: f64, y: f64) -> bool {
        let r = self.robot_radius;
        if x - r < self.x_min || x + r > self.x_max || y - r < self.y_min || y + r > self.y_max { return true; }
        self.boxes.iter().any(|b| {
            let ddx = (x - b.cx).abs() - b.w / 2.0;
            let ddy = (y - b.cy).abs() - b.h / 2.0;
            ddx.max(0.0).powi(2) + ddy.max(0.0).powi(2) < r * r
        })
    }
}
