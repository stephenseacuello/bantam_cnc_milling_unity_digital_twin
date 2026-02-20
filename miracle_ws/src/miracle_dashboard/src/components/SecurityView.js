import React, { useState, useEffect, useCallback, useMemo } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Chip,
  Button,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Grid,
  LinearProgress,
  IconButton,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Stack,
  Tooltip,
  Divider,
} from '@mui/material';
import ShieldIcon from '@mui/icons-material/Shield';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import DevicesIcon from '@mui/icons-material/Devices';
import EventNoteIcon from '@mui/icons-material/EventNote';
import VerifiedUserIcon from '@mui/icons-material/VerifiedUser';
import GppBadIcon from '@mui/icons-material/GppBad';
import LockIcon from '@mui/icons-material/Lock';
import RefreshIcon from '@mui/icons-material/Refresh';
import BlockIcon from '@mui/icons-material/Block';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import LinkOffIcon from '@mui/icons-material/LinkOff';
import {
  AreaChart, Area, XAxis, YAxis, CartesianGrid, Tooltip as RTooltip,
  ResponsiveContainer, PieChart, Pie, Cell, Legend,
} from 'recharts';

import {
  SECURITY_ALERTS,
  ISOLATE_NODE,
  DEFAULT_MACHINE_IDS,
} from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Severity / threat-level styling (mirrors AlertPanel patterns)
// ---------------------------------------------------------------------------
const SEVERITY = {
  CRITICAL: { color: '#f44336', bg: 'rgba(244,67,54,0.15)' },
  HIGH:     { color: '#ff9800', bg: 'rgba(255,152,0,0.15)' },
  MEDIUM:   { color: '#fdd835', bg: 'rgba(253,216,53,0.15)' },
  LOW:      { color: '#29b6f6', bg: 'rgba(41,182,246,0.15)' },
};
const sev = (s) => SEVERITY[(s || '').toUpperCase()] || SEVERITY.LOW;

const THREAT_LEVELS = {
  NORMAL:   { color: '#66bb6a', label: 'NORMAL' },
  ELEVATED: { color: '#fdd835', label: 'ELEVATED' },
  HIGH:     { color: '#ff9800', label: 'HIGH' },
  CRITICAL: { color: '#f44336', label: 'CRITICAL' },
};

const ROLE_COLORS = ['#4fc3f7', '#ff9800', '#66bb6a', '#ab47bc', '#ef5350'];

// ---------------------------------------------------------------------------
// Helper: format timestamp
// ---------------------------------------------------------------------------
const fmtTime = (iso) => {
  if (!iso) return '--';
  const d = new Date(iso);
  return d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
};

// ---------------------------------------------------------------------------
// Summary card component
// ---------------------------------------------------------------------------
function SummaryCard({ icon, title, value, color, subtitle }) {
  return (
    <Card sx={{ height: '100%' }}>
      <CardContent sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
        <Box sx={{
          width: 52, height: 52, borderRadius: 2,
          display: 'flex', alignItems: 'center', justifyContent: 'center',
          bgcolor: `${color}22`,
        }}>
          {React.cloneElement(icon, { sx: { fontSize: 28, color } })}
        </Box>
        <Box>
          <Typography variant="body2" color="text.secondary">{title}</Typography>
          <Typography variant="h5" sx={{ fontWeight: 700, color }}>{value}</Typography>
          {subtitle && (
            <Typography variant="caption" color="text.secondary">{subtitle}</Typography>
          )}
        </Box>
      </CardContent>
    </Card>
  );
}

// ---------------------------------------------------------------------------
// SecurityView
// ---------------------------------------------------------------------------
export default function SecurityView({ ros }) {
  const { subscribe, callService, connected } = ros;

  // -- State ---------------------------------------------------------------
  const [alerts, setAlerts] = useState([]);
  const [rateData, setRateData] = useState([]);
  const [auditLog, setAuditLog] = useState([]);
  const [auditFilterActor, setAuditFilterActor] = useState('');
  const [auditFilterAction, setAuditFilterAction] = useState('ALL');
  const [chainStatus, setChainStatus] = useState('VALID');

  // Simulated live state (populated from ROS messages when available)
  const [devices, setDevices] = useState(() =>
    DEFAULT_MACHINE_IDS.map((id) => ({
      id,
      trustScore: 0.85 + Math.random() * 0.15,
      attestation: 'VERIFIED',
      lastAttestation: new Date().toISOString(),
    })),
  );

  const [sessions] = useState([
    { user: 'admin',    role: 'engineer', permissions: 'full',       since: new Date().toISOString() },
    { user: 'op_floor', role: 'operator', permissions: 'read+submit', since: new Date().toISOString() },
    { user: 'viewer01', role: 'viewer',   permissions: 'read-only',  since: new Date().toISOString() },
  ]);

  const [sros2Active] = useState(true);
  const [encryptedTopics] = useState(14);
  const [isolatedNodes, setIsolatedNodes] = useState(new Set());

  // -- Subscribe to security alerts ----------------------------------------
  const onAlert = useCallback((msg) => {
    const entry = { ...msg, receivedAt: new Date().toISOString() };
    setAlerts((prev) => [entry, ...prev].slice(0, 500));

    // Mirror to audit log
    setAuditLog((prev) => [{
      timestamp: new Date().toISOString(),
      actor: msg.source_node || 'system',
      action: msg.category || 'ALERT',
      resource: (msg.affected_nodes || []).join(', ') || '--',
      result: msg.severity || 'INFO',
      hash: msg.alert_id || '',
    }, ...prev].slice(0, 1000));
  }, []);

  useEffect(() => {
    if (!connected || !subscribe) return;
    const sub = subscribe(SECURITY_ALERTS, 'miracle_msgs/msg/SecurityAlert', onAlert);
    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, onAlert]);

  // -- Alert-rate bucketing (per minute, last 60 minutes) ------------------
  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();
      const oneMinuteAgo = now - 60_000;
      const recentCount = alerts.filter(
        (a) => new Date(a.receivedAt).getTime() > oneMinuteAgo,
      ).length;

      setRateData((prev) => {
        const next = [...prev, { time: fmtTime(new Date().toISOString()), count: recentCount }];
        return next.length > 60 ? next.slice(-60) : next;
      });
    }, 60_000);

    // Seed initial point
    setRateData((prev) =>
      prev.length === 0
        ? [{ time: fmtTime(new Date().toISOString()), count: 0 }]
        : prev,
    );

    return () => clearInterval(interval);
  }, [alerts]);

  // -- Derived metrics -----------------------------------------------------
  const threatLevel = useMemo(() => {
    const critical = alerts.filter((a) => a.severity === 'CRITICAL').length;
    const high = alerts.filter((a) => a.severity === 'HIGH').length;
    if (critical > 0) return 'CRITICAL';
    if (high > 3) return 'HIGH';
    if (high > 0 || alerts.length > 10) return 'ELEVATED';
    return 'NORMAL';
  }, [alerts]);

  const tl = THREAT_LEVELS[threatLevel];

  const trustedCount = devices.filter((d) => d.trustScore > 0.5).length;

  const last24h = useMemo(() => {
    const cutoff = Date.now() - 86_400_000;
    return alerts.filter((a) => new Date(a.receivedAt).getTime() > cutoff).length;
  }, [alerts]);

  const accessDenials = useMemo(
    () => alerts.filter((a) => a.category === 'ACCESS_VIOLATION'),
    [alerts],
  );

  const roleDistribution = useMemo(() => {
    const counts = {};
    sessions.forEach((s) => { counts[s.role] = (counts[s.role] || 0) + 1; });
    return Object.entries(counts).map(([name, value]) => ({ name, value }));
  }, [sessions]);

  // -- Filtered audit log --------------------------------------------------
  const filteredAudit = useMemo(() => {
    return auditLog.filter((e) => {
      if (auditFilterActor && !e.actor.toLowerCase().includes(auditFilterActor.toLowerCase())) return false;
      if (auditFilterAction !== 'ALL' && e.action !== auditFilterAction) return false;
      return true;
    });
  }, [auditLog, auditFilterActor, auditFilterAction]);

  const uniqueActions = useMemo(
    () => [...new Set(auditLog.map((e) => e.action))],
    [auditLog],
  );

  // -- Service calls -------------------------------------------------------
  const handleReAttest = useCallback(async (deviceId) => {
    try {
      await callService(
        '/miracle/security/request_attestation',
        'miracle_msgs/srv/RequestAttestation',
        { device_id: deviceId },
      );
      setDevices((prev) =>
        prev.map((d) =>
          d.id === deviceId
            ? { ...d, attestation: 'PENDING', lastAttestation: new Date().toISOString() }
            : d,
        ),
      );
    } catch {
      // Service may not be available in dev mode -- update UI optimistically
      setDevices((prev) =>
        prev.map((d) =>
          d.id === deviceId
            ? { ...d, attestation: 'PENDING', lastAttestation: new Date().toISOString() }
            : d,
        ),
      );
    }
  }, [callService]);

  const handleIsolateNode = useCallback(async (nodeId) => {
    try {
      await callService(ISOLATE_NODE, 'miracle_msgs/srv/IsolateNode', { node_id: nodeId });
    } catch { /* best effort */ }
    setIsolatedNodes((prev) => new Set(prev).add(nodeId));
  }, [callService]);

  const handleVerifyChain = useCallback(() => {
    // Simulate chain verification
    setChainStatus(auditLog.length > 0 ? 'VALID' : 'EMPTY');
  }, [auditLog]);

  // -- Trust score color ---------------------------------------------------
  const trustColor = (score) => {
    if (score > 0.8) return '#66bb6a';
    if (score > 0.5) return '#fdd835';
    return '#f44336';
  };

  const attestChip = (status) => {
    const map = {
      VERIFIED: { icon: <VerifiedUserIcon />, color: 'success' },
      PENDING:  { icon: <RefreshIcon />,      color: 'warning' },
      FAILED:   { icon: <GppBadIcon />,       color: 'error' },
    };
    const cfg = map[status] || map.PENDING;
    return <Chip icon={cfg.icon} label={status} size="small" color={cfg.color} variant="outlined" />;
  };

  // -----------------------------------------------------------------------
  // Render
  // -----------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>

      {/* ====== 1. Summary Cards ========================================= */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        <Grid item xs={12} sm={6} md={3}>
          <SummaryCard
            icon={<ShieldIcon />}
            title="Threat Level"
            value={tl.label}
            color={tl.color}
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <SummaryCard
            icon={<WarningAmberIcon />}
            title="Active Alerts"
            value={alerts.length}
            color="#ff9800"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <SummaryCard
            icon={<DevicesIcon />}
            title="Trusted Devices"
            value={`${trustedCount} / ${devices.length}`}
            color="#4fc3f7"
            subtitle="trust score > 0.5"
          />
        </Grid>
        <Grid item xs={12} sm={6} md={3}>
          <SummaryCard
            icon={<EventNoteIcon />}
            title="Events (24 h)"
            value={last24h}
            color="#66bb6a"
          />
        </Grid>
      </Grid>

      {/* ====== 2. Intrusion Detection ==================================== */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {/* Recent alerts table */}
        <Grid item xs={12} md={7}>
          <Card>
            <CardContent sx={{ pb: 0 }}>
              <Typography variant="h6" gutterBottom>Intrusion Detection - Live Feed</Typography>
            </CardContent>
            <TableContainer sx={{ maxHeight: 320 }}>
              <Table stickyHeader size="small">
                <TableHead>
                  <TableRow>
                    <TableCell sx={{ fontWeight: 700 }}>Time</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Source</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Type</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Severity</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Description</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Action</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {alerts.length === 0 ? (
                    <TableRow>
                      <TableCell colSpan={6} align="center">
                        <Typography variant="body2" color="text.secondary" sx={{ py: 3 }}>
                          No alerts received yet. Listening on {SECURITY_ALERTS}
                        </Typography>
                      </TableCell>
                    </TableRow>
                  ) : (
                    alerts.slice(0, 50).map((a, i) => {
                      const s = sev(a.severity);
                      return (
                        <TableRow key={a.alert_id || i} sx={{ borderLeft: `4px solid ${s.color}` }}>
                          <TableCell sx={{ whiteSpace: 'nowrap' }}>{fmtTime(a.receivedAt)}</TableCell>
                          <TableCell>{a.source_node || '--'}</TableCell>
                          <TableCell>{a.category || '--'}</TableCell>
                          <TableCell>
                            <Chip
                              label={a.severity}
                              size="small"
                              sx={{ bgcolor: s.color, color: '#000', fontWeight: 700, fontSize: '0.7rem' }}
                            />
                          </TableCell>
                          <TableCell sx={{ maxWidth: 220, whiteSpace: 'normal' }}>
                            {a.description || '--'}
                          </TableCell>
                          <TableCell sx={{ maxWidth: 160, whiteSpace: 'normal' }}>
                            {a.recommended_action || '--'}
                          </TableCell>
                        </TableRow>
                      );
                    })
                  )}
                </TableBody>
              </Table>
            </TableContainer>
          </Card>
        </Grid>

        {/* Alert rate chart */}
        <Grid item xs={12} md={5}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Alert Rate (per minute)</Typography>
              <ResponsiveContainer width="100%" height={270}>
                <AreaChart data={rateData}>
                  <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.08)" />
                  <XAxis dataKey="time" tick={{ fontSize: 11, fill: '#aaa' }} />
                  <YAxis tick={{ fontSize: 11, fill: '#aaa' }} allowDecimals={false} />
                  <RTooltip contentStyle={{ backgroundColor: '#1e1e3a', border: 'none' }} />
                  <Area
                    type="monotone"
                    dataKey="count"
                    stroke="#f44336"
                    fill="rgba(244,67,54,0.25)"
                    strokeWidth={2}
                  />
                </AreaChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* ====== 3. Device Trust Scores ==================================== */}
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>Device Trust Scores</Typography>
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell sx={{ fontWeight: 700 }}>Device</TableCell>
                <TableCell sx={{ fontWeight: 700 }}>Trust Score</TableCell>
                <TableCell sx={{ fontWeight: 700 }} align="center">Attestation</TableCell>
                <TableCell sx={{ fontWeight: 700 }}>Last Attested</TableCell>
                <TableCell sx={{ fontWeight: 700 }} align="center">Action</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {devices.map((d) => (
                <TableRow key={d.id}>
                  <TableCell>{d.id}</TableCell>
                  <TableCell sx={{ minWidth: 200 }}>
                    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                      <LinearProgress
                        variant="determinate"
                        value={d.trustScore * 100}
                        sx={{
                          flexGrow: 1, height: 8, borderRadius: 4,
                          bgcolor: 'rgba(255,255,255,0.08)',
                          '& .MuiLinearProgress-bar': { bgcolor: trustColor(d.trustScore), borderRadius: 4 },
                        }}
                      />
                      <Typography variant="body2" sx={{ color: trustColor(d.trustScore), fontWeight: 700, minWidth: 40 }}>
                        {d.trustScore.toFixed(2)}
                      </Typography>
                    </Box>
                  </TableCell>
                  <TableCell align="center">{attestChip(d.attestation)}</TableCell>
                  <TableCell>{fmtTime(d.lastAttestation)}</TableCell>
                  <TableCell align="center">
                    <Button
                      size="small"
                      variant="outlined"
                      startIcon={<RefreshIcon />}
                      onClick={() => handleReAttest(d.id)}
                      disabled={d.attestation === 'PENDING'}
                    >
                      Re-Attest
                    </Button>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </CardContent>
      </Card>

      {/* ====== 4. Access Control ========================================= */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {/* Active sessions + denials */}
        <Grid item xs={12} md={8}>
          <Card sx={{ mb: 2 }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Active Sessions</Typography>
              <Table size="small">
                <TableHead>
                  <TableRow>
                    <TableCell sx={{ fontWeight: 700 }}>User</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Role</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Permissions</TableCell>
                    <TableCell sx={{ fontWeight: 700 }}>Since</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {sessions.map((s, i) => (
                    <TableRow key={i}>
                      <TableCell>{s.user}</TableCell>
                      <TableCell>
                        <Chip label={s.role} size="small" variant="outlined" color="primary" />
                      </TableCell>
                      <TableCell>{s.permissions}</TableCell>
                      <TableCell>{fmtTime(s.since)}</TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            </CardContent>
          </Card>

          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Recent Access Denials
                <Chip label={accessDenials.length} size="small" sx={{ ml: 1 }} color="error" variant="outlined" />
              </Typography>
              <TableContainer sx={{ maxHeight: 200 }}>
                <Table size="small">
                  <TableHead>
                    <TableRow>
                      <TableCell sx={{ fontWeight: 700 }}>Time</TableCell>
                      <TableCell sx={{ fontWeight: 700 }}>Role</TableCell>
                      <TableCell sx={{ fontWeight: 700 }}>Resource</TableCell>
                      <TableCell sx={{ fontWeight: 700 }}>Detail</TableCell>
                    </TableRow>
                  </TableHead>
                  <TableBody>
                    {accessDenials.length === 0 ? (
                      <TableRow>
                        <TableCell colSpan={4} align="center">
                          <Typography variant="body2" color="text.secondary" sx={{ py: 2 }}>
                            No access denials recorded.
                          </Typography>
                        </TableCell>
                      </TableRow>
                    ) : (
                      accessDenials.slice(0, 20).map((d, i) => (
                        <TableRow key={i} sx={{ borderLeft: '4px solid #fdd835' }}>
                          <TableCell>{fmtTime(d.receivedAt)}</TableCell>
                          <TableCell>{d.source_node || '--'}</TableCell>
                          <TableCell>{(d.affected_nodes || []).join(', ') || '--'}</TableCell>
                          <TableCell>{d.description || '--'}</TableCell>
                        </TableRow>
                      ))
                    )}
                  </TableBody>
                </Table>
              </TableContainer>
            </CardContent>
          </Card>
        </Grid>

        {/* Role distribution pie */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Role Distribution</Typography>
              <ResponsiveContainer width="100%" height={280}>
                <PieChart>
                  <Pie
                    data={roleDistribution}
                    cx="50%"
                    cy="50%"
                    innerRadius={50}
                    outerRadius={90}
                    dataKey="value"
                    label={({ name, value }) => `${name} (${value})`}
                  >
                    {roleDistribution.map((_, i) => (
                      <Cell key={i} fill={ROLE_COLORS[i % ROLE_COLORS.length]} />
                    ))}
                  </Pie>
                  <Legend verticalAlign="bottom" />
                  <RTooltip contentStyle={{ backgroundColor: '#1e1e3a', border: 'none' }} />
                </PieChart>
              </ResponsiveContainer>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* ====== 5. Audit Log Viewer ======================================= */}
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Stack direction="row" alignItems="center" justifyContent="space-between" sx={{ mb: 2 }}>
            <Typography variant="h6">Audit Log (Tamper-Evident)</Typography>
            <Stack direction="row" spacing={1} alignItems="center">
              <Chip
                icon={chainStatus === 'VALID' ? <CheckCircleIcon /> : <GppBadIcon />}
                label={`Chain: ${chainStatus}`}
                size="small"
                color={chainStatus === 'VALID' ? 'success' : 'error'}
                variant="outlined"
              />
              <Button size="small" variant="outlined" startIcon={<LockIcon />} onClick={handleVerifyChain}>
                Verify Chain
              </Button>
            </Stack>
          </Stack>

          {/* Filters */}
          <Stack direction="row" spacing={2} sx={{ mb: 2 }} flexWrap="wrap" useFlexGap>
            <TextField
              label="Filter by actor"
              size="small"
              value={auditFilterActor}
              onChange={(e) => setAuditFilterActor(e.target.value)}
              sx={{ minWidth: 180 }}
            />
            <FormControl size="small" sx={{ minWidth: 160 }}>
              <InputLabel>Action type</InputLabel>
              <Select
                value={auditFilterAction}
                label="Action type"
                onChange={(e) => setAuditFilterAction(e.target.value)}
              >
                <MenuItem value="ALL">All</MenuItem>
                {uniqueActions.map((a) => (
                  <MenuItem key={a} value={a}>{a}</MenuItem>
                ))}
              </Select>
            </FormControl>
            <Chip
              label={`${filteredAudit.length} entries`}
              size="small"
              variant="outlined"
              sx={{ alignSelf: 'center' }}
            />
          </Stack>

          <TableContainer sx={{ maxHeight: 300 }}>
            <Table stickyHeader size="small">
              <TableHead>
                <TableRow>
                  <TableCell sx={{ fontWeight: 700 }}>Timestamp</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Actor</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Action</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Resource</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Result</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {filteredAudit.length === 0 ? (
                  <TableRow>
                    <TableCell colSpan={5} align="center">
                      <Typography variant="body2" color="text.secondary" sx={{ py: 3 }}>
                        No audit entries match the current filters.
                      </Typography>
                    </TableCell>
                  </TableRow>
                ) : (
                  filteredAudit.slice(0, 100).map((e, i) => (
                    <TableRow key={i}>
                      <TableCell sx={{ whiteSpace: 'nowrap' }}>{fmtTime(e.timestamp)}</TableCell>
                      <TableCell>{e.actor}</TableCell>
                      <TableCell>
                        <Chip label={e.action} size="small" variant="outlined" />
                      </TableCell>
                      <TableCell>{e.resource}</TableCell>
                      <TableCell>
                        <Chip
                          label={e.result}
                          size="small"
                          sx={{ bgcolor: sev(e.result).color, color: '#000', fontWeight: 700, fontSize: '0.7rem' }}
                        />
                      </TableCell>
                    </TableRow>
                  ))
                )}
              </TableBody>
            </Table>
          </TableContainer>
        </CardContent>
      </Card>

      {/* ====== 6. Network Security ======================================= */}
      <Card>
        <CardContent>
          <Typography variant="h6" gutterBottom>Network Security - DDS / SROS2</Typography>
          <Grid container spacing={3}>
            <Grid item xs={12} sm={4}>
              <Stack spacing={1} alignItems="center">
                <Chip
                  icon={sros2Active ? <LockIcon /> : <GppBadIcon />}
                  label={sros2Active ? 'SROS2 Active' : 'SROS2 Inactive'}
                  color={sros2Active ? 'success' : 'error'}
                  sx={{ fontWeight: 700, fontSize: '0.85rem', py: 2, px: 1 }}
                />
                <Typography variant="body2" color="text.secondary">
                  {encryptedTopics} encrypted topics
                </Typography>
              </Stack>
            </Grid>

            <Grid item xs={12} sm={8}>
              <Typography variant="subtitle2" gutterBottom>Node Isolation Controls</Typography>
              <Table size="small">
                <TableHead>
                  <TableRow>
                    <TableCell sx={{ fontWeight: 700 }}>Node</TableCell>
                    <TableCell sx={{ fontWeight: 700 }} align="center">Status</TableCell>
                    <TableCell sx={{ fontWeight: 700 }} align="center">Action</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {DEFAULT_MACHINE_IDS.map((id) => {
                    const isolated = isolatedNodes.has(id);
                    return (
                      <TableRow key={id}>
                        <TableCell>{id}</TableCell>
                        <TableCell align="center">
                          <Chip
                            icon={isolated ? <LinkOffIcon /> : <CheckCircleIcon />}
                            label={isolated ? 'ISOLATED' : 'CONNECTED'}
                            size="small"
                            color={isolated ? 'error' : 'success'}
                            variant="outlined"
                          />
                        </TableCell>
                        <TableCell align="center">
                          <Tooltip title={isolated ? 'Node already isolated' : `Isolate ${id} from the network`}>
                            <span>
                              <IconButton
                                size="small"
                                color="error"
                                disabled={isolated}
                                onClick={() => handleIsolateNode(id)}
                              >
                                <BlockIcon />
                              </IconButton>
                            </span>
                          </Tooltip>
                        </TableCell>
                      </TableRow>
                    );
                  })}
                </TableBody>
              </Table>
            </Grid>
          </Grid>
        </CardContent>
      </Card>
    </Box>
  );
}
