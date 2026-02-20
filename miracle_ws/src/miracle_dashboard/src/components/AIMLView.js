import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  LinearProgress,
  Chip,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Divider,
} from '@mui/material';
import ErrorIcon from '@mui/icons-material/Error';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import BugReportIcon from '@mui/icons-material/BugReport';
import BuildIcon from '@mui/icons-material/Build';
import HubIcon from '@mui/icons-material/Hub';
import ModelTrainingIcon from '@mui/icons-material/ModelTraining';
import MemoryIcon from '@mui/icons-material/Memory';
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip,
  ResponsiveContainer,
} from 'recharts';

import {
  anomalyTopic,
  phmPredictionTopic,
  toolWearTopic,
  GLOBAL_MODEL,
  MODEL_UPDATES,
  DEFAULT_MACHINE_IDS,
} from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
const severityColor = (sev) => {
  const s = (sev || '').toUpperCase();
  if (s === 'CRITICAL') return '#f44336';
  if (s === 'HIGH') return '#ffa726';
  if (s === 'MEDIUM') return '#fdd835';
  return '#29b6f6';
};

const riskChip = (rul) => {
  if (rul <= 24) return { label: 'CRITICAL', color: 'error' };
  if (rul <= 168) return { label: 'HIGH', color: 'warning' };
  if (rul <= 720) return { label: 'MEDIUM', color: 'info' };
  return { label: 'LOW', color: 'success' };
};

const scoreColor = (v) => {
  if (v > 0.7) return '#f44336';
  if (v > 0.4) return '#ffa726';
  return '#66bb6a';
};

const wearGradient = (pct) => {
  if (pct > 80) return '#f44336';
  if (pct > 50) return '#ffa726';
  return '#66bb6a';
};

const fmtTime = (stamp) => {
  if (!stamp) return '--';
  try {
    const sec = stamp.sec ?? stamp.secs ?? 0;
    return new Date(sec * 1000).toLocaleTimeString();
  } catch {
    return '--';
  }
};

const cardSx = { height: '100%' };

// ---------------------------------------------------------------------------
// AIMLView
// ---------------------------------------------------------------------------
export default function AIMLView({ ros }) {
  const { subscribe, connected } = ros;

  // -- Anomaly state -------------------------------------------------------
  const [anomalies, setAnomalies] = useState({}); // machineId -> latest msg
  const [anomalyLog, setAnomalyLog] = useState([]);

  // -- PHM state -----------------------------------------------------------
  const [phm, setPhm] = useState({});            // machineId -> latest msg
  const [healthTrend, setHealthTrend] = useState([]); // [{time, cnc_01, ...}]

  // -- Tool wear state -----------------------------------------------------
  const [toolWear, setToolWear] = useState({});   // machineId -> latest msg

  // -- Federated learning --------------------------------------------------
  const [fedModel, setFedModel] = useState(null);

  // -- Model registry ------------------------------------------------------
  const [models, setModels] = useState([]);

  // -----------------------------------------------------------------------
  // Subscriptions
  // -----------------------------------------------------------------------

  // Anomaly topics per machine
  useEffect(() => {
    if (!connected || !subscribe) return;
    const subs = DEFAULT_MACHINE_IDS.map((id) =>
      subscribe(anomalyTopic(id), 'miracle_msgs/msg/AnomalyAlert', (msg) => {
        setAnomalies((prev) => ({ ...prev, [id]: msg }));
        setAnomalyLog((prev) => [{ ...msg, machine_id: msg.machine_id || id }, ...prev].slice(0, 30));
      }),
    );
    return () => subs.forEach((s) => s && s.unsubscribe());
  }, [connected, subscribe]);

  // PHM predictions per machine
  useEffect(() => {
    if (!connected || !subscribe) return;
    const subs = DEFAULT_MACHINE_IDS.map((id) =>
      subscribe(phmPredictionTopic(id), 'miracle_msgs/msg/PHMPrediction', (msg) => {
        setPhm((prev) => ({ ...prev, [id]: msg }));
        setHealthTrend((prev) => {
          const point = { time: new Date().toLocaleTimeString() };
          DEFAULT_MACHINE_IDS.forEach((mid) => {
            point[mid] = mid === id ? (msg.health_index ?? 0) : (prev.length > 0 ? (prev[prev.length - 1][mid] ?? null) : null);
          });
          return [...prev, point].slice(-30);
        });
      }),
    );
    return () => subs.forEach((s) => s && s.unsubscribe());
  }, [connected, subscribe]);

  // Tool wear per machine
  useEffect(() => {
    if (!connected || !subscribe) return;
    const subs = DEFAULT_MACHINE_IDS.map((id) =>
      subscribe(toolWearTopic(id), 'miracle_msgs/msg/ToolWearEstimate', (msg) => {
        setToolWear((prev) => ({ ...prev, [id]: msg }));
      }),
    );
    return () => subs.forEach((s) => s && s.unsubscribe());
  }, [connected, subscribe]);

  // Federated learning global model
  useEffect(() => {
    if (!connected || !subscribe) return;
    const sub = subscribe(GLOBAL_MODEL, 'miracle_msgs/msg/FederatedModel', (msg) => {
      setFedModel(msg);
    });
    return () => sub && sub.unsubscribe();
  }, [connected, subscribe]);

  // Model registry updates
  const onModelUpdate = useCallback((msg) => {
    setModels((prev) => {
      const idx = prev.findIndex((m) => m.model_name === msg.model_name);
      if (idx >= 0) {
        const next = [...prev];
        next[idx] = msg;
        return next;
      }
      return [...prev, msg].slice(-20);
    });
  }, []);

  useEffect(() => {
    if (!connected || !subscribe) return;
    const sub = subscribe(MODEL_UPDATES, 'miracle_msgs/msg/ModelUpdate', onModelUpdate);
    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, onModelUpdate]);

  // -----------------------------------------------------------------------
  // Derived values
  // -----------------------------------------------------------------------
  const activeAnomalyCount = Object.values(anomalies).filter(
    (a) => (a.confidence ?? 0) > 0.5,
  ).length;

  const LINE_COLORS = ['#4fc3f7', '#ff9800', '#66bb6a'];

  // -----------------------------------------------------------------------
  // Render
  // -----------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h5" gutterBottom>AI / ML Intelligence</Typography>

      {/* ---- Row 1: Anomaly Detection ----------------------------------- */}
      <Typography variant="h6" sx={{ mb: 1 }}>Anomaly Detection</Typography>
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {/* Per-machine anomaly scores */}
        <Grid item xs={12} md={5}>
          <Card sx={cardSx}>
            <CardContent>
              <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
                <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>Machine Anomaly Status</Typography>
                <Chip
                  icon={<BugReportIcon />}
                  label={`${activeAnomalyCount} active`}
                  color={activeAnomalyCount > 0 ? 'error' : 'success'}
                  size="small"
                />
              </Box>
              {DEFAULT_MACHINE_IDS.map((id) => {
                const a = anomalies[id];
                const score = a?.confidence ?? 0;
                return (
                  <Box key={id} sx={{ mb: 1.5 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
                      <Typography variant="body2">{id}</Typography>
                      <Typography variant="body2" sx={{ color: scoreColor(score), fontWeight: 600 }}>
                        {score.toFixed(2)}
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={Math.min(score * 100, 100)}
                      sx={{
                        height: 8, borderRadius: 4,
                        bgcolor: 'rgba(255,255,255,0.08)',
                        '& .MuiLinearProgress-bar': { bgcolor: scoreColor(score), borderRadius: 4 },
                      }}
                    />
                    {a?.anomaly_type && (
                      <Typography variant="caption" color="text.secondary">
                        {a.anomaly_type} | {a.recommended_action || '--'}
                      </Typography>
                    )}
                  </Box>
                );
              })}
            </CardContent>
          </Card>
        </Grid>

        {/* Recent anomaly list */}
        <Grid item xs={12} md={7}>
          <Card sx={cardSx}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, mb: 1 }}>Recent Anomalies</Typography>
              {anomalyLog.length === 0 ? (
                <Typography variant="body2" color="text.secondary">No anomalies detected yet.</Typography>
              ) : (
                <List dense disablePadding sx={{ maxHeight: 220, overflow: 'auto' }}>
                  {anomalyLog.slice(0, 8).map((a, idx) => (
                    <React.Fragment key={idx}>
                      {idx > 0 && <Divider component="li" />}
                      <ListItem disableGutters>
                        <ListItemIcon sx={{ minWidth: 32 }}>
                          <WarningAmberIcon sx={{ color: severityColor(a.severity > 0.8 ? 'CRITICAL' : a.severity > 0.5 ? 'HIGH' : 'MEDIUM'), fontSize: 20 }} />
                        </ListItemIcon>
                        <ListItemText
                          primary={`${a.machine_id} - ${a.anomaly_type || 'UNKNOWN'} (${(a.confidence ?? 0).toFixed(2)})`}
                          secondary={`${fmtTime(a.timestamp)} | ${a.recommended_action || '--'}`}
                          primaryTypographyProps={{ variant: 'body2' }}
                          secondaryTypographyProps={{ variant: 'caption' }}
                        />
                      </ListItem>
                    </React.Fragment>
                  ))}
                </List>
              )}
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* ---- Row 2: PHM & Tool Wear ------------------------------------- */}
      <Typography variant="h6" sx={{ mb: 1 }}>Predictive Health Management</Typography>
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {/* RUL cards */}
        <Grid item xs={12} md={4}>
          <Card sx={cardSx}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, mb: 2 }}>Remaining Useful Life</Typography>
              {DEFAULT_MACHINE_IDS.map((id) => {
                const p = phm[id];
                const rul = p?.remaining_useful_life_hours ?? 1000;
                const risk = riskChip(rul);
                const health = p?.health_index ?? 1.0;
                return (
                  <Box key={id} sx={{ mb: 2, p: 1.5, borderRadius: 2, bgcolor: 'rgba(255,255,255,0.03)' }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                      <Typography variant="body2" sx={{ fontWeight: 600 }}>{id}</Typography>
                      <Chip label={risk.label} color={risk.color} size="small" />
                    </Box>
                    <Typography variant="h5" sx={{ fontWeight: 700, mt: 0.5 }}>
                      {rul.toFixed(0)} <Typography component="span" variant="caption" color="text.secondary">hrs</Typography>
                    </Typography>
                    <Typography variant="caption" color="text.secondary">
                      {p?.component || 'spindle_bearing'} | Health: {(health * 100).toFixed(0)}%
                    </Typography>
                    <LinearProgress
                      variant="determinate"
                      value={Math.min(health * 100, 100)}
                      sx={{
                        height: 6, borderRadius: 3, mt: 0.5,
                        bgcolor: 'rgba(255,255,255,0.08)',
                        '& .MuiLinearProgress-bar': {
                          bgcolor: health > 0.6 ? '#66bb6a' : health > 0.3 ? '#ffa726' : '#f44336',
                          borderRadius: 3,
                        },
                      }}
                    />
                  </Box>
                );
              })}
            </CardContent>
          </Card>
        </Grid>

        {/* Health degradation trend chart */}
        <Grid item xs={12} md={4}>
          <Card sx={cardSx}>
            <CardContent>
              <Typography variant="subtitle1" sx={{ fontWeight: 600, mb: 1 }}>Health Degradation Trend</Typography>
              {healthTrend.length < 2 ? (
                <Typography variant="body2" color="text.secondary">Awaiting trend data...</Typography>
              ) : (
                <ResponsiveContainer width="100%" height={220}>
                  <LineChart data={healthTrend}>
                    <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.08)" />
                    <XAxis dataKey="time" tick={{ fill: '#aaa', fontSize: 10 }} />
                    <YAxis domain={[0, 1]} tick={{ fill: '#aaa', fontSize: 10 }} />
                    <Tooltip contentStyle={{ backgroundColor: '#1e1e3f', border: 'none', borderRadius: 8 }} />
                    {DEFAULT_MACHINE_IDS.map((id, i) => (
                      <Line key={id} type="monotone" dataKey={id} stroke={LINE_COLORS[i % LINE_COLORS.length]} dot={false} strokeWidth={2} />
                    ))}
                  </LineChart>
                </ResponsiveContainer>
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* Tool wear monitoring */}
        <Grid item xs={12} md={4}>
          <Card sx={cardSx}>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                <BuildIcon sx={{ color: 'secondary.main' }} />
                <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>Tool Wear Monitoring</Typography>
              </Box>
              {DEFAULT_MACHINE_IDS.map((id) => {
                const tw = toolWear[id];
                const pct = tw?.wear_percentage ?? 0;
                const remaining = tw?.estimated_remaining_life_hours ?? '--';
                return (
                  <Box key={id} sx={{ mb: 1.5 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
                      <Typography variant="body2">{id} {tw?.tool_id ? `(${tw.tool_id})` : ''}</Typography>
                      <Typography variant="body2" sx={{ color: wearGradient(pct), fontWeight: 600 }}>
                        {pct.toFixed(0)}%
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={Math.min(pct, 100)}
                      sx={{
                        height: 8, borderRadius: 4,
                        bgcolor: 'rgba(255,255,255,0.08)',
                        '& .MuiLinearProgress-bar': { bgcolor: wearGradient(pct), borderRadius: 4 },
                      }}
                    />
                    <Typography variant="caption" color="text.secondary">
                      {tw?.wear_type || '--'} | Est. remaining: {typeof remaining === 'number' ? `${remaining.toFixed(0)} hrs` : remaining}
                    </Typography>
                  </Box>
                );
              })}
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* ---- Row 3: Federated Learning & Model Registry ----------------- */}
      <Grid container spacing={2}>
        {/* Federated learning status */}
        <Grid item xs={12} md={4}>
          <Card sx={cardSx}>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                <HubIcon sx={{ color: 'primary.main' }} />
                <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>Federated Learning</Typography>
              </Box>
              {!fedModel ? (
                <Typography variant="body2" color="text.secondary">Awaiting global model data...</Typography>
              ) : (
                <>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                    <Typography variant="body2" color="text.secondary">Global Model Version</Typography>
                    <Typography variant="body2" sx={{ fontWeight: 600 }}>v{fedModel.version ?? '--'}</Typography>
                  </Box>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                    <Typography variant="body2" color="text.secondary">Accuracy</Typography>
                    <Typography variant="body2" sx={{ fontWeight: 600, color: '#66bb6a' }}>
                      {((fedModel.accuracy ?? 0) * 100).toFixed(1)}%
                    </Typography>
                  </Box>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                    <Typography variant="body2" color="text.secondary">Participants</Typography>
                    <Typography variant="body2" sx={{ fontWeight: 600 }}>{fedModel.participating_machines ?? '--'}</Typography>
                  </Box>
                  <Divider sx={{ my: 1.5 }} />
                  <Typography variant="body2" color="text.secondary" sx={{ mb: 0.5 }}>Round Progress</Typography>
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                    <LinearProgress
                      variant="determinate"
                      value={Math.min((fedModel.round_progress ?? 0) * 100, 100)}
                      sx={{
                        flexGrow: 1, height: 8, borderRadius: 4,
                        bgcolor: 'rgba(255,255,255,0.08)',
                        '& .MuiLinearProgress-bar': { bgcolor: '#4fc3f7', borderRadius: 4 },
                      }}
                    />
                    <Typography variant="body2" sx={{ fontWeight: 600 }}>
                      {((fedModel.round_progress ?? 0) * 100).toFixed(0)}%
                    </Typography>
                  </Box>
                  <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                    Round {fedModel.current_round ?? '--'} of {fedModel.total_rounds ?? '--'}
                  </Typography>
                </>
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* Model registry table */}
        <Grid item xs={12} md={8}>
          <Card sx={cardSx}>
            <CardContent>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                <ModelTrainingIcon sx={{ color: 'secondary.main' }} />
                <Typography variant="subtitle1" sx={{ fontWeight: 600 }}>Model Registry</Typography>
              </Box>
              {models.length === 0 ? (
                <Typography variant="body2" color="text.secondary">No model updates received yet.</Typography>
              ) : (
                <TableContainer sx={{ maxHeight: 260 }}>
                  <Table size="small" stickyHeader>
                    <TableHead>
                      <TableRow>
                        <TableCell sx={{ bgcolor: 'background.paper', fontWeight: 600 }}>Model</TableCell>
                        <TableCell sx={{ bgcolor: 'background.paper', fontWeight: 600 }}>Version</TableCell>
                        <TableCell sx={{ bgcolor: 'background.paper', fontWeight: 600 }}>Type</TableCell>
                        <TableCell sx={{ bgcolor: 'background.paper', fontWeight: 600 }}>Accuracy</TableCell>
                        <TableCell sx={{ bgcolor: 'background.paper', fontWeight: 600 }}>Status</TableCell>
                      </TableRow>
                    </TableHead>
                    <TableBody>
                      {models.map((m, idx) => {
                        const status = (m.status || 'unknown').toLowerCase();
                        const chipColor = status === 'active' ? 'success' : status === 'training' ? 'info' : 'default';
                        return (
                          <TableRow key={m.model_name || idx} hover>
                            <TableCell>
                              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                                <MemoryIcon sx={{ fontSize: 16, color: 'primary.main' }} />
                                {m.model_name || '--'}
                              </Box>
                            </TableCell>
                            <TableCell>{m.version || '--'}</TableCell>
                            <TableCell>{m.model_type || '--'}</TableCell>
                            <TableCell>{((m.accuracy ?? 0) * 100).toFixed(1)}%</TableCell>
                            <TableCell>
                              <Chip label={status} color={chipColor} size="small" variant="outlined" />
                            </TableCell>
                          </TableRow>
                        );
                      })}
                    </TableBody>
                  </Table>
                </TableContainer>
              )}
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
}
