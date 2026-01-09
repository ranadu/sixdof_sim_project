import { useMemo, useState } from "react";
import Plot from "react-plotly.js";

type RunResponse = {
  ok: boolean;
  task: string;
  result: {
    t: number[];
    X: number[][]; // [N x 13]
    F: number[][]; // [N x 3]
    M: number[][]; // [N x 3]
    meta?: Record<string, any>;
  };
};

const API_BASE = import.meta.env.VITE_API_BASE as string;

function col(X: number[][], idx: number) {
  return X.map((r) => r[idx]);
}

export default function App() {
  const [loading, setLoading] = useState(false);
  const [err, setErr] = useState<string | null>(null);
  const [data, setData] = useState<RunResponse | null>(null);

  const [scenario, setScenario] = useState<"aileron_pulse" | "elevator_step" | "trim">("aileron_pulse");

  const payload = useMemo(() => {
    // state vector: pos(0:3), vel_b(3:6), q(6:10), omega(10:13)
    if (scenario === "trim") {
      return {
        dt: 0.01,
        t_final: 12.0,
        downsample_hz: 50,
        state0: {
          pos_ned: [0, 0, 1000],
          vel_b: [65, 0, 0],
          euler_rad: [0, 0, 0],
          omega_b: [0, 0, 0],
        },
        controls: { type: "constant", throttle: 0.6, de: 0.0, da: 0.0, dr: 0.0 },
      };
    }

    if (scenario === "elevator_step") {
      return {
        dt: 0.01,
        t_final: 12.0,
        downsample_hz: 50,
        state0: {
          pos_ned: [0, 0, 1000],
          vel_b: [65, 0, 0],
          euler_rad: [0, 0, 0],
          omega_b: [0, 0, 0],
        },
        controls: { type: "elevator_step", throttle: 0.6, de: 0.0, magnitude: -0.08, t_on: 2.0 },
      };
    }

    // default: aileron pulse
    return {
      dt: 0.01,
      t_final: 12.0,
      downsample_hz: 50,
      state0: {
        pos_ned: [0, 0, 1000],
        vel_b: [65, 0, 0],
        euler_rad: [0, 0, 0],
        omega_b: [0, 0, 0],
      },
      controls: { type: "aileron_pulse", throttle: 0.6, magnitude: 0.17, t_on: 2.0, t_off: 3.0 },
    };
  }, [scenario]);

  async function runSim() {
    setErr(null);
    setLoading(true);
    setData(null);

    try {
      const res = await fetch(`${API_BASE}/api/v1/run`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ task: "sixdof_simulate", mode: "sync", payload }),
      });

      if (!res.ok) {
        const text = await res.text();
        throw new Error(`Backend error ${res.status}: ${text}`);
      }

      const json = (await res.json()) as RunResponse;
      setData(json);
    } catch (e: any) {
      setErr(e?.message ?? String(e));
    } finally {
      setLoading(false);
    }
  }

  const t = data?.result.t ?? [];
  const X = data?.result.X ?? [];
  const F = data?.result.F ?? [];
  const M = data?.result.M ?? [];

  // Extract basic signals
  const pn = col(X, 0);
  const pe = col(X, 1);
  const pd = col(X, 2);

  const u = col(X, 3);
  const v = col(X, 4);
  const w = col(X, 5);

  const q0 = col(X, 6);
  const q1 = col(X, 7);
  const q2 = col(X, 8);
  const q3 = col(X, 9);

  const pRate = col(X, 10);
  const qRate = col(X, 11);
  const rRate = col(X, 12);

  // Simple derived approximations (for display)
  // Speed in body
  const V = X.map((r) => Math.sqrt(r[3] * r[3] + r[4] * r[4] + r[5] * r[5]));

  return (
    <div style={{ fontFamily: "system-ui", padding: 18, maxWidth: 1200, margin: "0 auto" }}>
      <h1 style={{ marginBottom: 6 }}>6-DOF Aircraft Simulator (Render + Vercel)</h1>
      <p style={{ marginTop: 0, opacity: 0.8 }}>
        Backend: <code>{API_BASE}</code>
      </p>

      <div
        style={{
          display: "flex",
          gap: 12,
          alignItems: "center",
          flexWrap: "wrap",
          padding: 12,
          border: "1px solid #ddd",
          borderRadius: 12,
          marginBottom: 16,
        }}
      >
        <label>
          Scenario:&nbsp;
          <select value={scenario} onChange={(e) => setScenario(e.target.value as any)}>
            <option value="aileron_pulse">Aileron pulse (roll response)</option>
            <option value="elevator_step">Elevator step (pitch response)</option>
            <option value="trim">Trim-ish (hands off)</option>
          </select>
        </label>

        <button
          onClick={runSim}
          disabled={loading}
          style={{
            padding: "10px 14px",
            borderRadius: 10,
            border: "1px solid #999",
            cursor: loading ? "not-allowed" : "pointer",
          }}
        >
          {loading ? "Running..." : "Run Simulation"}
        </button>

        {err && (
          <div style={{ color: "crimson", fontWeight: 600, whiteSpace: "pre-wrap" }}>
            {err}
          </div>
        )}

        {data && (
          <div style={{ opacity: 0.8 }}>
            samples: <b>{t.length}</b> | dt: <b>{data.result.meta?.dt}</b> | stride:{" "}
            <b>{data.result.meta?.stride}</b>
          </div>
        )}
      </div>

      {!data ? (
        <div style={{ opacity: 0.75 }}>
          Click <b>Run Simulation</b> to pull telemetry from the backend and plot it.
        </div>
      ) : (
        <div style={{ display: "grid", gridTemplateColumns: "1fr", gap: 18 }}>
          <Plot
            data={[
              { x: t, y: pn, type: "scatter", name: "pN (m)" },
              { x: t, y: pe, type: "scatter", name: "pE (m)" },
              { x: t, y: pd, type: "scatter", name: "pD (m)" },
            ]}
            layout={{ title: "Position (NED)", height: 360, margin: { l: 60, r: 20, t: 50, b: 50 } }}
            style={{ width: "100%" }}
          />

          <Plot
            data={[
              { x: t, y: u, type: "scatter", name: "u (m/s)" },
              { x: t, y: v, type: "scatter", name: "v (m/s)" },
              { x: t, y: w, type: "scatter", name: "w (m/s)" },
              { x: t, y: V, type: "scatter", name: "|V| (m/s)" },
            ]}
            layout={{ title: "Body Velocity", height: 360, margin: { l: 60, r: 20, t: 50, b: 50 } }}
            style={{ width: "100%" }}
          />

          <Plot
            data={[
              { x: t, y: pRate, type: "scatter", name: "p (rad/s)" },
              { x: t, y: qRate, type: "scatter", name: "q (rad/s)" },
              { x: t, y: rRate, type: "scatter", name: "r (rad/s)" },
            ]}
            layout={{ title: "Body Rates", height: 360, margin: { l: 60, r: 20, t: 50, b: 50 } }}
            style={{ width: "100%" }}
          />

          <Plot
            data={[
              { x: t, y: q0, type: "scatter", name: "q0" },
              { x: t, y: q1, type: "scatter", name: "q1" },
              { x: t, y: q2, type: "scatter", name: "q2" },
              { x: t, y: q3, type: "scatter", name: "q3" },
            ]}
            layout={{ title: "Quaternion (body→NED)", height: 360, margin: { l: 60, r: 20, t: 50, b: 50 } }}
            style={{ width: "100%" }}
          />

          <Plot
            data={[
              { x: t, y: F.map((r) => r[0]), type: "scatter", name: "Fx (N)" },
              { x: t, y: F.map((r) => r[1]), type: "scatter", name: "Fy (N)" },
              { x: t, y: F.map((r) => r[2]), type: "scatter", name: "Fz (N)" },
            ]}
            layout={{ title: "Forces (body)", height: 360, margin: { l: 60, r: 20, t: 50, b: 50 } }}
            style={{ width: "100%" }}
          />

          <Plot
            data={[
              { x: t, y: M.map((r) => r[0]), type: "scatter", name: "Mx (N·m)" },
              { x: t, y: M.map((r) => r[1]), type: "scatter", name: "My (N·m)" },
              { x: t, y: M.map((r) => r[2]), type: "scatter", name: "Mz (N·m)" },
            ]}
            layout={{ title: "Moments (body)", height: 360, margin: { l: 60, r: 20, t: 50, b: 50 } }}
            style={{ width: "100%" }}
          />
        </div>
      )}

      <div style={{ marginTop: 22, opacity: 0.75, fontSize: 13 }}>
        Tip: If the browser blocks the request, we’ll set CORS to your Vercel domain in Render env var <code>ALLOWED_ORIGINS</code>.
      </div>
    </div>
  );
}