import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  LinearProgress,
  Table,
  TableBody,
  TableCell,
  TableRow,
} from '@mui/material';
import {
  AreaChart,
  Area,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from 'recharts';

import { SYSTEM_KPIS } from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const MAX_HISTORY = 120; // ~2 min of KPI history at 1 Hz

const METRIC_COLORS = {
  oee:          '#ffa726',
  availability: '#66bb6a',
  performance:  '#29b6f6',
  quality:      '#ab47bc',
};

// ---------------------------------------------------------------------------
// Metric bar component
// ---------------------------------------------------------------------------
function MetricBar({ label, value, color }) {
  const pct = Math.min((value || 0) * 100, 100);
  return (
    <Box sx={{ mb: 2 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
        <Typography variant="body2" sx={{ fontWeight: 600 }}>
          {label}
        </Typography>
        <Typography variant="body2" sx={{ color, fontWeight: 700 }}>
          {pct.toFixed(1)}%
        </Typography>
      </Box>
      <LinearProgress
        variant="determinate"
        value={pct}
        sx={{
          height: 12,
          borderRadius: 6,
          bgcolor: 'rgba(255,255,255,0.08)',
          '& .MuiLinearProgress-bar': { bgcolor: color, borderRadius: 6 },
        }}
      />
    </Box>
  );
}

// ---------------------------------------------------------------------------
// OEEPanel component
// ---------------------------------------------------------------------------
export default function OEEPanel({ ros }) {
  const { subscribe, connected } = ros;

  const [kpis, setKpis] = useState(null);
  const [history, setHistory] = useState([]);

  // ------------------------------------------------------------------
  // Subscribe to system KPIs
  // ------------------------------------------------------------------
  const onKpis = useCallback((msg) => {
    setKpis(msg);

    const point = {
      time: new Date().toLocaleTimeString(),
      oee:          parseFloat(((msg.oee || 0) * 100).toFixed(1)),
      availability: parseFloat(((msg.availability || 0) * 100).toFixed(1)),
      performance:  parseFloat(((msg.performance || 0) * 100).toFixed(1)),
      quality:      parseFloat(((msg.quality || 0) * 100).toFixed(1)),
    };

    setHistory((prev) => [...prev.slice(-(MAX_HISTORY - 1)), point]);
  }, []);

  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      SYSTEM_KPIS,
      'miracle_msgs/msg/SystemKPIs',
      onKpis,
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, onKpis]);

  // ------------------------------------------------------------------
  // Derived values
  // ------------------------------------------------------------------
  const oee          = kpis?.oee ?? 0;
  const availability = kpis?.availability ?? 0;
  const performance  = kpis?.performance ?? 0;
  const quality      = kpis?.quality ?? 0;

  // ------------------------------------------------------------------
  // Render
  // ------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>
      {/* Real-time metric display */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {/* Overall OEE highlight */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent sx={{ textAlign: 'center', py: 4 }}>
              <Typography variant="subtitle1" color="text.secondary">
                Overall OEE
              </Typography>
              <Typography
                variant="h2"
                sx={{ fontWeight: 700, color: METRIC_COLORS.oee, my: 1 }}
              >
                {(oee * 100).toFixed(1)}%
              </Typography>
              <Typography variant="body2" color="text.secondary">
                {oee >= 0.85 ? 'World-class' : oee >= 0.65 ? 'Typical' : 'Needs improvement'}
              </Typography>
            </CardContent>
          </Card>
        </Grid>

        {/* Breakdown bars */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>OEE Breakdown</Typography>
              <MetricBar label="Availability" value={availability} color={METRIC_COLORS.availability} />
              <MetricBar label="Performance"  value={performance}  color={METRIC_COLORS.performance} />
              <MetricBar label="Quality"      value={quality}      color={METRIC_COLORS.quality} />
            </CardContent>
          </Card>
        </Grid>

        {/* Additional KPI details */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Additional KPIs</Typography>
              <Table size="small">
                <TableBody>
                  <TableRow>
                    <TableCell component="th">Cpk</TableCell>
                    <TableCell align="right">
                      {kpis?.cpk?.toFixed(2) ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">MTBF</TableCell>
                    <TableCell align="right">
                      {kpis?.mtbf != null ? `${kpis.mtbf.toFixed(1)} h` : '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">MTTR</TableCell>
                    <TableCell align="right">
                      {kpis?.mttr != null ? `${kpis.mttr.toFixed(1)} h` : '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Energy Efficiency</TableCell>
                    <TableCell align="right">
                      {kpis?.energy_efficiency != null
                        ? `${(kpis.energy_efficiency * 100).toFixed(1)}%`
                        : '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Schedule Adherence</TableCell>
                    <TableCell align="right">
                      {kpis?.schedule_adherence != null
                        ? `${(kpis.schedule_adherence * 100).toFixed(1)}%`
                        : '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Tool Life Utilization</TableCell>
                    <TableCell align="right">
                      {kpis?.tool_life_utilization != null
                        ? `${(kpis.tool_life_utilization * 100).toFixed(1)}%`
                        : '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Jobs Completed Today</TableCell>
                    <TableCell align="right">
                      {kpis?.jobs_completed_today ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Jobs In Progress</TableCell>
                    <TableCell align="right">
                      {kpis?.jobs_in_progress ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Jobs Queued</TableCell>
                    <TableCell align="right">
                      {kpis?.jobs_queued ?? '--'}
                    </TableCell>
                  </TableRow>
                </TableBody>
              </Table>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Historical OEE trend chart */}
      <Card>
        <CardContent>
          <Typography variant="h6" gutterBottom>OEE Trend (real-time)</Typography>
          {history.length < 2 ? (
            <Typography variant="body2" color="text.secondary" sx={{ py: 4, textAlign: 'center' }}>
              Collecting KPI data... The trend chart will appear once at least two data points arrive.
            </Typography>
          ) : (
            <ResponsiveContainer width="100%" height={320}>
              <AreaChart data={history}>
                <defs>
                  <linearGradient id="gradOEE" x1="0" y1="0" x2="0" y2="1">
                    <stop offset="5%" stopColor={METRIC_COLORS.oee} stopOpacity={0.3} />
                    <stop offset="95%" stopColor={METRIC_COLORS.oee} stopOpacity={0} />
                  </linearGradient>
                  <linearGradient id="gradAvail" x1="0" y1="0" x2="0" y2="1">
                    <stop offset="5%" stopColor={METRIC_COLORS.availability} stopOpacity={0.2} />
                    <stop offset="95%" stopColor={METRIC_COLORS.availability} stopOpacity={0} />
                  </linearGradient>
                  <linearGradient id="gradPerf" x1="0" y1="0" x2="0" y2="1">
                    <stop offset="5%" stopColor={METRIC_COLORS.performance} stopOpacity={0.2} />
                    <stop offset="95%" stopColor={METRIC_COLORS.performance} stopOpacity={0} />
                  </linearGradient>
                  <linearGradient id="gradQual" x1="0" y1="0" x2="0" y2="1">
                    <stop offset="5%" stopColor={METRIC_COLORS.quality} stopOpacity={0.2} />
                    <stop offset="95%" stopColor={METRIC_COLORS.quality} stopOpacity={0} />
                  </linearGradient>
                </defs>
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.06)" />
                <XAxis
                  dataKey="time"
                  tick={{ fontSize: 10 }}
                  interval="preserveStartEnd"
                />
                <YAxis
                  domain={[0, 100]}
                  tick={{ fontSize: 10 }}
                  unit="%"
                  width={45}
                />
                <Tooltip
                  contentStyle={{
                    background: '#1e1e3a',
                    border: 'none',
                    borderRadius: 8,
                    fontSize: 12,
                  }}
                  formatter={(value) => `${value}%`}
                />
                <Legend
                  wrapperStyle={{ fontSize: 12 }}
                />
                <Area
                  type="monotone"
                  dataKey="oee"
                  name="OEE"
                  stroke={METRIC_COLORS.oee}
                  fill="url(#gradOEE)"
                  strokeWidth={2}
                  isAnimationActive={false}
                />
                <Area
                  type="monotone"
                  dataKey="availability"
                  name="Availability"
                  stroke={METRIC_COLORS.availability}
                  fill="url(#gradAvail)"
                  strokeWidth={1.5}
                  isAnimationActive={false}
                />
                <Area
                  type="monotone"
                  dataKey="performance"
                  name="Performance"
                  stroke={METRIC_COLORS.performance}
                  fill="url(#gradPerf)"
                  strokeWidth={1.5}
                  isAnimationActive={false}
                />
                <Area
                  type="monotone"
                  dataKey="quality"
                  name="Quality"
                  stroke={METRIC_COLORS.quality}
                  fill="url(#gradQual)"
                  strokeWidth={1.5}
                  isAnimationActive={false}
                />
              </AreaChart>
            </ResponsiveContainer>
          )}
        </CardContent>
      </Card>
    </Box>
  );
}
