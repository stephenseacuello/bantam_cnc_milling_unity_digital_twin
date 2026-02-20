import React, { useState, useEffect, useCallback, useRef } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  LinearProgress,
  Chip,
  Button,
  Divider,
  Tooltip,
} from '@mui/material';
import SyncIcon from '@mui/icons-material/Sync';
import SyncDisabledIcon from '@mui/icons-material/SyncDisabled';
import SyncProblemIcon from '@mui/icons-material/SyncProblem';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import ErrorOutlineIcon from '@mui/icons-material/ErrorOutline';
import HourglassBottomIcon from '@mui/icons-material/HourglassBottom';


import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  Legend,
  ReferenceLine,
  ResponsiveContainer,
} from 'recharts';

import { DEFAULT_MACHINE_IDS, machineStateTopic } from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const DRIFT_THRESHOLD = 1.0;
const MAX_DRIFT_HISTORY = 60;
const AXIS_LABELS = ['X', 'Y', 'Z'];
const MACHINE_COLORS = { cnc1: '#4fc3f7', cnc2: '#ff9800', cnc3: '#66bb6a' };

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
const twinStateTopic = (id) => `/miracle/${id}/twin_state`;
const SYNC_STATUS_TOPIC = '/miracle/twin/sync_status';
const RUN_PREDICTION_SERVICE = '/miracle/twin/run_prediction';

const syncStatusLabel = (quality) => {
  if (quality > 0.95) return 'SYNCED';
  if (quality > 0.80) return 'DRIFTING';
  return 'DISCONNECTED';
};

const syncStatusColor = (quality) => {
  if (quality > 0.95) return '#66bb6a';
  if (quality > 0.80) return '#ffa726';
  return '#f44336';
};

const syncStatusIcon = (quality) => {
  if (quality > 0.95) return <SyncIcon sx={{ color: '#66bb6a' }} />;
  if (quality > 0.80) return <SyncProblemIcon sx={{ color: '#ffa726' }} />;
  return <SyncDisabledIcon sx={{ color: '#f44336' }} />;
};

const formatTimestamp = (epoch) => {
  if (!epoch) return '--:--:--';
  const d = new Date(epoch * 1000);
  return d.toLocaleTimeString();
};

// ---------------------------------------------------------------------------
// AxisBar: animated horizontal bar comparing physical vs twin value
// ---------------------------------------------------------------------------
function AxisBar({ label, physical, twin, max }) {
  const clamp = (v) => Math.min(Math.max(v, 0), max);
  const physPct = (clamp(physical) / max) * 100;
  const twinPct = (clamp(twin) / max) * 100;

  return (
    <Box sx={{ mb: 1.5 }}>
      <Typography variant="caption" color="text.secondary">{label}</Typography>
      <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
        <Typography variant="caption" sx={{ width: 48, textAlign: 'right', color: '#4fc3f7' }}>
          {physical?.toFixed(1) ?? '--'}
        </Typography>
        <Box sx={{ flex: 1 }}>
          <LinearProgress
            variant="determinate"
            value={physPct}
            sx={{
              height: 6, borderRadius: 3, bgcolor: 'rgba(255,255,255,0.06)',
              '& .MuiLinearProgress-bar': {
                bgcolor: '#4fc3f7', borderRadius: 3,
                transition: 'transform 0.4s ease',
              },
            }}
          />
          <LinearProgress
            variant="determinate"
            value={twinPct}
            sx={{
              height: 6, borderRadius: 3, mt: 0.5, bgcolor: 'rgba(255,255,255,0.06)',
              '& .MuiLinearProgress-bar': {
                bgcolor: '#ab47bc', borderRadius: 3,
                transition: 'transform 0.4s ease',
              },
            }}
          />
        </Box>
        <Typography variant="caption" sx={{ width: 48, textAlign: 'right', color: '#ab47bc' }}>
          {twin?.toFixed(1) ?? '--'}
        </Typography>
      </Box>
    </Box>
  );
}

// ---------------------------------------------------------------------------
// DigitalTwinView component
// ---------------------------------------------------------------------------
export default function DigitalTwinView({ ros }) {
  const { subscribe, callService, connected } = ros;

  const [selectedMachine, setSelectedMachine] = useState(DEFAULT_MACHINE_IDS[0]);
  const [physicalStates, setPhysicalStates] = useState({});
  const [twinStates, setTwinStates] = useState({});
  const [syncStatus, setSyncStatus] = useState({});
  const [driftHistory, setDriftHistory] = useState([]);
  const [predictions, setPredictions] = useState([]);
  const [predictionLoading, setPredictionLoading] = useState(false);
  const driftRef = useRef([]);

  // -----------------------------------------------------------------------
  // Subscribe to physical machine states
  // -----------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const subs = DEFAULT_MACHINE_IDS.map((id) =>
      subscribe(
        machineStateTopic(id),
        'miracle_msgs/msg/MachineState',
        (msg) => {
          setPhysicalStates((prev) => ({ ...prev, [id]: msg }));
        },
      ),
    );

    return () => subs.forEach((s) => s && s.unsubscribe());
  }, [connected, subscribe]);

  // -----------------------------------------------------------------------
  // Subscribe to twin states
  // -----------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const subs = DEFAULT_MACHINE_IDS.map((id) =>
      subscribe(
        twinStateTopic(id),
        'miracle_msgs/msg/MachineState',
        (msg) => {
          setTwinStates((prev) => ({ ...prev, [id]: msg }));
        },
      ),
    );

    return () => subs.forEach((s) => s && s.unsubscribe());
  }, [connected, subscribe]);

  // -----------------------------------------------------------------------
  // Subscribe to sync status
  // -----------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      SYNC_STATUS_TOPIC,
      'miracle_msgs/msg/TwinSyncStatus',
      (msg) => {
        if (msg.machine_statuses) {
          const statusMap = {};
          msg.machine_statuses.forEach((entry) => {
            statusMap[entry.machine_id] = {
              sync_quality: entry.sync_quality ?? 1.0,
              drift_magnitude: entry.drift_magnitude ?? 0.0,
              last_sync_time: entry.last_sync_time ?? 0.0,
            };
          });
          setSyncStatus(statusMap);
        }
      },
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe]);

  // -----------------------------------------------------------------------
  // Build drift history from sync status updates
  // -----------------------------------------------------------------------
  useEffect(() => {
    if (Object.keys(syncStatus).length === 0) return;

    const now = new Date();
    const point = { time: now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }) };
    DEFAULT_MACHINE_IDS.forEach((id) => {
      point[id] = syncStatus[id]?.drift_magnitude ?? null;
    });

    const updated = [...driftRef.current, point].slice(-MAX_DRIFT_HISTORY);
    driftRef.current = updated;
    setDriftHistory(updated);
  }, [syncStatus]);

  // -----------------------------------------------------------------------
  // Trigger prediction via ROS service
  // -----------------------------------------------------------------------
  const handleRunPrediction = useCallback(async () => {
    if (!callService) return;
    setPredictionLoading(true);

    const scenario = {
      name: `Prediction ${predictions.length + 1}`,
      machine_id: selectedMachine,
      status: 'running',
      outcome: 'Pending...',
      timestamp: new Date().toLocaleTimeString(),
    };
    setPredictions((prev) => [scenario, ...prev].slice(0, 10));

    try {
      const result = await callService(
        RUN_PREDICTION_SERVICE,
        'miracle_msgs/srv/RunPrediction',
        { machine_id: selectedMachine, scenario_type: 'what_if' },
      );

      setPredictions((prev) =>
        prev.map((p) =>
          p.name === scenario.name
            ? { ...p, status: 'completed', outcome: result.summary || 'Analysis complete' }
            : p,
        ),
      );
    } catch (err) {
      setPredictions((prev) =>
        prev.map((p) =>
          p.name === scenario.name
            ? { ...p, status: 'failed', outcome: err.message || 'Service unavailable' }
            : p,
        ),
      );
    } finally {
      setPredictionLoading(false);
    }
  }, [callService, selectedMachine, predictions.length]);

  // -----------------------------------------------------------------------
  // Derived data for selected machine
  // -----------------------------------------------------------------------
  const phys = physicalStates[selectedMachine];
  const twin = twinStates[selectedMachine];
  const physAxes = phys?.axis_positions || [];
  const twinAxes = twin?.axis_positions || [];

  // -----------------------------------------------------------------------
  // Render
  // -----------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h5" gutterBottom sx={{ fontWeight: 700 }}>
        Digital Twin Synchronization
      </Typography>

      {/* ---- Section 1: Twin Sync Status Cards ---- */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {DEFAULT_MACHINE_IDS.map((id) => {
          const ss = syncStatus[id] || {};
          const quality = ss.sync_quality ?? 1.0;
          const drift = ss.drift_magnitude ?? 0.0;
          const qualityPct = (quality * 100).toFixed(1);
          const isSelected = id === selectedMachine;

          return (
            <Grid item xs={12} sm={6} md={4} key={id}>
              <Card
                onClick={() => setSelectedMachine(id)}
                sx={{
                  cursor: 'pointer',
                  border: isSelected ? '2px solid' : '1px solid rgba(255,255,255,0.06)',
                  borderColor: isSelected ? 'primary.main' : 'rgba(255,255,255,0.06)',
                  transition: 'border-color 0.2s',
                  '&:hover': { borderColor: 'primary.main' },
                }}
              >
                <CardContent>
                  <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 1 }}>
                    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                      {syncStatusIcon(quality)}
                      <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>
                        {id.toUpperCase().replace('_', ' ')}
                      </Typography>
                    </Box>
                    <Chip
                      label={syncStatusLabel(quality)}
                      size="small"
                      sx={{
                        bgcolor: `${syncStatusColor(quality)}22`,
                        color: syncStatusColor(quality),
                        fontWeight: 600,
                        fontSize: '0.7rem',
                      }}
                    />
                  </Box>

                  <Box sx={{ display: 'flex', alignItems: 'baseline', gap: 0.5, mb: 0.5 }}>
                    <Typography variant="h4" sx={{ fontWeight: 700, color: syncStatusColor(quality) }}>
                      {qualityPct}%
                    </Typography>
                    <Typography variant="body2" color="text.secondary">sync</Typography>
                  </Box>

                  <LinearProgress
                    variant="determinate"
                    value={Math.min(quality * 100, 100)}
                    sx={{
                      height: 6, borderRadius: 3, mb: 1.5,
                      bgcolor: 'rgba(255,255,255,0.06)',
                      '& .MuiLinearProgress-bar': {
                        bgcolor: syncStatusColor(quality), borderRadius: 3,
                      },
                    }}
                  />

                  <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                    <Typography variant="caption" color="text.secondary">
                      Drift: <b style={{ color: drift > DRIFT_THRESHOLD ? '#f44336' : '#66bb6a' }}>
                        {drift.toFixed(4)}
                      </b>
                      {drift > DRIFT_THRESHOLD && ' (!)'}
                    </Typography>
                    <Typography variant="caption" color="text.secondary">
                      {formatTimestamp(ss.last_sync_time)}
                    </Typography>
                  </Box>
                </CardContent>
              </Card>
            </Grid>
          );
        })}
      </Grid>

      {/* ---- Section 2: Live Position Visualization ---- */}
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 2 }}>
            <Typography variant="h6">
              Live State: {selectedMachine.toUpperCase().replace('_', ' ')}
            </Typography>
            <Box sx={{ display: 'flex', gap: 1, alignItems: 'center' }}>
              <Chip label="Physical" size="small" sx={{ bgcolor: 'rgba(79,195,247,0.15)', color: '#4fc3f7' }} />
              <Chip label="Digital Twin" size="small" sx={{ bgcolor: 'rgba(171,71,188,0.15)', color: '#ab47bc' }} />
            </Box>
          </Box>

          <Grid container spacing={3}>
            <Grid item xs={12} md={6}>
              <Typography variant="subtitle2" color="text.secondary" gutterBottom>
                Axis Positions (mm)
              </Typography>
              {AXIS_LABELS.map((axis, i) => (
                <AxisBar
                  key={axis}
                  label={`${axis} Axis`}
                  physical={physAxes[i] ?? 0}
                  twin={twinAxes[i] ?? 0}
                  max={500}
                />
              ))}
            </Grid>

            <Grid item xs={12} md={6}>
              <Typography variant="subtitle2" color="text.secondary" gutterBottom>
                Spindle & Feed
              </Typography>
              <AxisBar
                label="Spindle Speed (RPM)"
                physical={phys?.spindle_speed ?? 0}
                twin={twin?.spindle_speed ?? 0}
                max={24000}
              />
              <AxisBar
                label="Feed Rate (mm/min)"
                physical={phys?.feed_rate ?? 0}
                twin={twin?.feed_rate ?? 0}
                max={10000}
              />
              <AxisBar
                label="Spindle Load (%)"
                physical={phys?.spindle_load ?? 0}
                twin={twin?.spindle_load ?? 0}
                max={100}
              />
            </Grid>
          </Grid>
        </CardContent>
      </Card>

      <Grid container spacing={2}>
        {/* ---- Section 3: Drift History Chart ---- */}
        <Grid item xs={12} md={7}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Drift History</Typography>
              {driftHistory.length === 0 ? (
                <Typography variant="body2" color="text.secondary" sx={{ py: 4, textAlign: 'center' }}>
                  Waiting for sync status data...
                </Typography>
              ) : (
                <ResponsiveContainer width="100%" height={260}>
                  <LineChart data={driftHistory} margin={{ top: 5, right: 20, bottom: 5, left: 0 }}>
                    <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.06)" />
                    <XAxis
                      dataKey="time"
                      tick={{ fill: '#888', fontSize: 11 }}
                      tickLine={false}
                      interval="preserveStartEnd"
                    />
                    <YAxis
                      tick={{ fill: '#888', fontSize: 11 }}
                      tickLine={false}
                      domain={[0, 'auto']}
                    />
                    <RechartsTooltip
                      contentStyle={{
                        backgroundColor: '#1a1a2e',
                        border: '1px solid rgba(255,255,255,0.12)',
                        borderRadius: 8,
                        fontSize: 12,
                      }}
                    />
                    <Legend wrapperStyle={{ fontSize: 12 }} />
                    <ReferenceLine
                      y={DRIFT_THRESHOLD}
                      stroke="#f44336"
                      strokeDasharray="6 3"
                      label={{ value: 'Threshold', fill: '#f44336', fontSize: 11, position: 'right' }}
                    />
                    {DEFAULT_MACHINE_IDS.map((id) => (
                      <Line
                        key={id}
                        type="monotone"
                        dataKey={id}
                        name={id.toUpperCase().replace('_', ' ')}
                        stroke={MACHINE_COLORS[id] || '#ccc'}
                        strokeWidth={2}
                        dot={false}
                        activeDot={{ r: 4 }}
                        connectNulls
                      />
                    ))}
                  </LineChart>
                </ResponsiveContainer>
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* ---- Section 4: Prediction Scenarios ---- */}
        <Grid item xs={12} md={5}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 2 }}>
                <Typography variant="h6">Prediction Scenarios</Typography>
                <Tooltip title="Run what-if prediction on selected machine">
                  <span>
                    <Button
                      variant="contained"
                      size="small"
                      startIcon={<PlayArrowIcon />}
                      disabled={predictionLoading || !connected}
                      onClick={handleRunPrediction}
                      sx={{ textTransform: 'none' }}
                    >
                      {predictionLoading ? 'Running...' : 'Run Prediction'}
                    </Button>
                  </span>
                </Tooltip>
              </Box>

              {predictions.length === 0 ? (
                <Typography variant="body2" color="text.secondary" sx={{ py: 3, textAlign: 'center' }}>
                  No predictions yet. Click "Run Prediction" to start a what-if analysis
                  on {selectedMachine.toUpperCase().replace('_', ' ')}.
                </Typography>
              ) : (
                <Box sx={{ maxHeight: 240, overflowY: 'auto' }}>
                  {predictions.map((pred, idx) => (
                    <React.Fragment key={pred.name}>
                      {idx > 0 && <Divider sx={{ my: 1 }} />}
                      <Box sx={{ display: 'flex', alignItems: 'flex-start', gap: 1.5, py: 0.5 }}>
                        {pred.status === 'running' && (
                          <HourglassBottomIcon sx={{ color: '#ffa726', fontSize: 20, mt: 0.3 }} />
                        )}
                        {pred.status === 'completed' && (
                          <CheckCircleIcon sx={{ color: '#66bb6a', fontSize: 20, mt: 0.3 }} />
                        )}
                        {pred.status === 'failed' && (
                          <ErrorOutlineIcon sx={{ color: '#f44336', fontSize: 20, mt: 0.3 }} />
                        )}
                        <Box sx={{ flex: 1, minWidth: 0 }}>
                          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                            <Typography variant="body2" sx={{ fontWeight: 600 }}>
                              {pred.name}
                            </Typography>
                            <Chip
                              label={pred.status}
                              size="small"
                              sx={{
                                fontSize: '0.65rem', height: 20,
                                bgcolor: pred.status === 'completed' ? 'rgba(102,187,106,0.15)'
                                  : pred.status === 'failed' ? 'rgba(244,67,54,0.15)'
                                  : 'rgba(255,167,38,0.15)',
                                color: pred.status === 'completed' ? '#66bb6a'
                                  : pred.status === 'failed' ? '#f44336'
                                  : '#ffa726',
                              }}
                            />
                          </Box>
                          <Typography variant="caption" color="text.secondary" noWrap>
                            {pred.machine_id} | {pred.timestamp}
                          </Typography>
                          <Typography variant="body2" color="text.secondary" sx={{ mt: 0.25 }}>
                            {pred.outcome}
                          </Typography>
                        </Box>
                      </Box>
                    </React.Fragment>
                  ))}
                </Box>
              )}
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
}
