import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  LinearProgress,
  Chip,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Divider,
} from '@mui/material';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import PlayCircleIcon from '@mui/icons-material/PlayCircle';
import PauseCircleIcon from '@mui/icons-material/PauseCircle';
import ErrorIcon from '@mui/icons-material/Error';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import WarningAmberIcon from '@mui/icons-material/WarningAmber';
import FiberManualRecordIcon from '@mui/icons-material/FiberManualRecord';

import {
  SYSTEM_KPIS,
  FLEET_HEALTH,
  SECURITY_ALERTS,
  DEFAULT_MACHINE_IDS,
  machineStateTopic,
} from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Helper: colour for a machine status string
// ---------------------------------------------------------------------------
const statusColor = (status) => {
  switch (status?.toUpperCase()) {
    case 'RUNNING':  return 'success';
    case 'IDLE':     return 'info';
    case 'ERROR':    return 'error';
    case 'E_STOP':   return 'error';
    default:         return 'default';
  }
};

// ---------------------------------------------------------------------------
// OEE Gauge - simple percentage arc rendered as a progress bar
// ---------------------------------------------------------------------------
function OEEGauge({ label, value, color }) {
  return (
    <Box sx={{ textAlign: 'center', px: 1 }}>
      <Typography variant="h4" sx={{ color, fontWeight: 700 }}>
        {(value * 100).toFixed(1)}%
      </Typography>
      <LinearProgress
        variant="determinate"
        value={Math.min(value * 100, 100)}
        sx={{
          height: 8,
          borderRadius: 4,
          mt: 1,
          bgcolor: 'rgba(255,255,255,0.08)',
          '& .MuiLinearProgress-bar': { bgcolor: color, borderRadius: 4 },
        }}
      />
      <Typography variant="body2" color="text.secondary" sx={{ mt: 0.5 }}>
        {label}
      </Typography>
    </Box>
  );
}

// ---------------------------------------------------------------------------
// Dashboard component
// ---------------------------------------------------------------------------
export default function Dashboard({ ros }) {
  const { subscribe, connected } = ros;

  // Fleet counts
  const [fleet, setFleet] = useState({
    total: DEFAULT_MACHINE_IDS.length,
    active: 0,
    idle: 0,
    error: 0,
  });

  // Machine statuses map
  const [machineStates, setMachineStates] = useState({});

  // KPIs
  const [kpis, setKpis] = useState({
    oee: 0,
    availability: 0,
    performance: 0,
    quality: 0,
    healthScore: 1.0,
  });

  // Recent alerts (last 10)
  const [recentAlerts, setRecentAlerts] = useState([]);

  // ------------------------------------------------------------------
  // Subscribe to machine states
  // ------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const subs = DEFAULT_MACHINE_IDS.map((id) =>
      subscribe(
        machineStateTopic(id),
        'miracle_msgs/msg/MachineState',
        (msg) => {
          setMachineStates((prev) => ({ ...prev, [id]: msg }));
        },
      ),
    );

    return () => subs.forEach((s) => s && s.unsubscribe());
  }, [connected, subscribe]);

  // Derive fleet counts from machine states
  useEffect(() => {
    const states = Object.values(machineStates);
    const total = Math.max(states.length, DEFAULT_MACHINE_IDS.length);
    let active = 0;
    let idle = 0;
    let errorCount = 0;

    states.forEach((s) => {
      const st = (s.status || '').toUpperCase();
      if (st === 'RUNNING') active++;
      else if (st === 'ERROR' || st === 'E_STOP') errorCount++;
      else idle++;
    });

    setFleet({ total, active, idle: total - active - errorCount, error: errorCount });
  }, [machineStates]);

  // ------------------------------------------------------------------
  // Subscribe to system KPIs
  // ------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      SYSTEM_KPIS,
      'miracle_msgs/msg/SystemKPIs',
      (msg) => {
        setKpis({
          oee: msg.oee || 0,
          availability: msg.availability || 0,
          performance: msg.performance || 0,
          quality: msg.quality || 0,
        });
      },
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe]);

  // ------------------------------------------------------------------
  // Subscribe to fleet health
  // ------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      FLEET_HEALTH,
      'miracle_msgs/msg/FleetHealth',
      (msg) => {
        setKpis((prev) => ({ ...prev, healthScore: msg.health_score ?? 1.0 }));
      },
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe]);

  // ------------------------------------------------------------------
  // Subscribe to security alerts (recent list)
  // ------------------------------------------------------------------
  const onAlert = useCallback((msg) => {
    setRecentAlerts((prev) => [msg, ...prev].slice(0, 10));
  }, []);

  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      SECURITY_ALERTS,
      'miracle_msgs/msg/SecurityAlert',
      onAlert,
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, onAlert]);

  // ------------------------------------------------------------------
  // Severity icon helper
  // ------------------------------------------------------------------
  const severityIcon = (sev) => {
    const s = (sev || '').toUpperCase();
    if (s === 'CRITICAL') return <ErrorIcon sx={{ color: 'error.main' }} />;
    if (s === 'HIGH') return <WarningAmberIcon sx={{ color: 'warning.main' }} />;
    if (s === 'MEDIUM') return <WarningAmberIcon sx={{ color: '#fdd835' }} />;
    return <FiberManualRecordIcon sx={{ color: 'info.main' }} />;
  };

  // ------------------------------------------------------------------
  // Render
  // ------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>
      {/* Fleet status cards */}
      <Grid container spacing={2} sx={{ mb: 3 }}>
        {[
          { label: 'Total Machines', value: fleet.total, icon: <PrecisionManufacturingIcon />, color: 'primary.main' },
          { label: 'Active',         value: fleet.active, icon: <PlayCircleIcon />,             color: 'success.main' },
          { label: 'Idle',           value: fleet.idle,   icon: <PauseCircleIcon />,            color: 'info.main' },
          { label: 'Error',          value: fleet.error,  icon: <ErrorIcon />,                  color: 'error.main' },
        ].map((card) => (
          <Grid item xs={12} sm={6} md={3} key={card.label}>
            <Card>
              <CardContent sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                <Box sx={{ color: card.color, fontSize: 40, display: 'flex' }}>
                  {card.icon}
                </Box>
                <Box>
                  <Typography variant="h4" sx={{ fontWeight: 700 }}>{card.value}</Typography>
                  <Typography variant="body2" color="text.secondary">{card.label}</Typography>
                </Box>
              </CardContent>
            </Card>
          </Grid>
        ))}
      </Grid>

      {/* OEE gauges */}
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>OEE Overview</Typography>
          <Grid container spacing={2}>
            <Grid item xs={6} md={3}>
              <OEEGauge label="Availability" value={kpis.availability} color="#66bb6a" />
            </Grid>
            <Grid item xs={6} md={3}>
              <OEEGauge label="Performance" value={kpis.performance} color="#29b6f6" />
            </Grid>
            <Grid item xs={6} md={3}>
              <OEEGauge label="Quality" value={kpis.quality} color="#ab47bc" />
            </Grid>
            <Grid item xs={6} md={3}>
              <OEEGauge label="Overall OEE" value={kpis.oee} color="#ffa726" />
            </Grid>
          </Grid>
        </CardContent>
      </Card>

      <Grid container spacing={2}>
        {/* Recent alerts */}
        <Grid item xs={12} md={7}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Recent Alerts</Typography>
              {recentAlerts.length === 0 ? (
                <Typography variant="body2" color="text.secondary">
                  No alerts received yet.
                </Typography>
              ) : (
                <List dense disablePadding>
                  {recentAlerts.map((alert, idx) => (
                    <React.Fragment key={alert.alert_id || idx}>
                      {idx > 0 && <Divider component="li" />}
                      <ListItem disableGutters>
                        <ListItemIcon sx={{ minWidth: 36 }}>
                          {severityIcon(alert.severity)}
                        </ListItemIcon>
                        <ListItemText
                          primary={alert.description || 'Alert'}
                          secondary={`${alert.severity || ''} | ${alert.source_node || 'unknown'}`}
                        />
                      </ListItem>
                    </React.Fragment>
                  ))}
                </List>
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* System health */}
        <Grid item xs={12} md={5}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>System Health</Typography>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mt: 2 }}>
                <CheckCircleIcon
                  sx={{
                    fontSize: 48,
                    color: kpis.healthScore > 0.8 ? 'success.main'
                      : kpis.healthScore > 0.5 ? 'warning.main'
                      : 'error.main',
                  }}
                />
                <Box>
                  <Typography variant="h4" sx={{ fontWeight: 700 }}>
                    {(kpis.healthScore * 100).toFixed(0)}%
                  </Typography>
                  <Typography variant="body2" color="text.secondary">
                    Fleet Health Score
                  </Typography>
                </Box>
              </Box>

              <Divider sx={{ my: 2 }} />

              <Typography variant="subtitle2" gutterBottom>Machine Status</Typography>
              <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
                {DEFAULT_MACHINE_IDS.map((id) => {
                  const st = machineStates[id]?.status || 'UNKNOWN';
                  return (
                    <Chip
                      key={id}
                      label={`${id}: ${st}`}
                      color={statusColor(st)}
                      size="small"
                      variant="outlined"
                    />
                  );
                })}
              </Box>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
}
