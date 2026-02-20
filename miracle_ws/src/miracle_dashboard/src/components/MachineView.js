import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Grid,
  Card,
  CardContent,
  Typography,
  Chip,
  LinearProgress,
  Select,
  MenuItem,
  FormControl,
  InputLabel,
  Divider,
  Table,
  TableBody,
  TableCell,
  TableRow,
} from '@mui/material';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from 'recharts';

import {
  DEFAULT_MACHINE_IDS,
  machineStateTopic,
  sensorDataTopic,
  toolWearTopic,
} from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const MAX_CHART_POINTS = 60; // ~60 s of history at 1 Hz

const STATE_COLORS = {
  RUNNING: 'success',
  IDLE:    'info',
  ERROR:   'error',
  E_STOP:  'error',
};

// ---------------------------------------------------------------------------
// MachineView component
// ---------------------------------------------------------------------------
export default function MachineView({ ros }) {
  const { subscribe, connected } = ros;

  const [selectedMachine, setSelectedMachine] = useState(DEFAULT_MACHINE_IDS[0]);
  const [machineState, setMachineState] = useState(null);
  const [toolWear, setToolWear] = useState(null);

  // Sensor time-series keyed by sensor_type
  const [sensorHistory, setSensorHistory] = useState({
    vibration:   [],
    current:     [],
    temperature: [],
  });

  // ------------------------------------------------------------------
  // Subscribe to machine state
  // ------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      machineStateTopic(selectedMachine),
      'miracle_msgs/msg/MachineState',
      (msg) => setMachineState(msg),
    );

    // Reset on machine change
    setMachineState(null);
    setSensorHistory({ vibration: [], current: [], temperature: [] });
    setToolWear(null);

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, selectedMachine]);

  // ------------------------------------------------------------------
  // Subscribe to sensor data
  // ------------------------------------------------------------------
  const onSensorData = useCallback((msg) => {
    const type = (msg.sensor_type || '').toLowerCase();
    if (!['vibration', 'current', 'temperature'].includes(type)) return;

    const value = msg.values?.[0] ?? 0;
    const point = {
      time: new Date().toLocaleTimeString(),
      value: parseFloat(value.toFixed(3)),
    };

    setSensorHistory((prev) => ({
      ...prev,
      [type]: [...prev[type].slice(-(MAX_CHART_POINTS - 1)), point],
    }));
  }, []);

  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      sensorDataTopic(selectedMachine),
      'miracle_msgs/msg/SensorData',
      onSensorData,
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, selectedMachine, onSensorData]);

  // ------------------------------------------------------------------
  // Subscribe to tool wear
  // ------------------------------------------------------------------
  useEffect(() => {
    if (!connected || !subscribe) return;

    const sub = subscribe(
      toolWearTopic(selectedMachine),
      'miracle_msgs/msg/ToolWearEstimate',
      (msg) => setToolWear(msg),
    );

    return () => sub && sub.unsubscribe();
  }, [connected, subscribe, selectedMachine]);

  // ------------------------------------------------------------------
  // Derived values
  // ------------------------------------------------------------------
  const status = machineState?.status?.toUpperCase() || 'UNKNOWN';
  const chipColor = STATE_COLORS[status] || 'default';

  const axisLabels = ['X', 'Y', 'Z', 'A', 'B', 'C'];
  const axisPositions = machineState?.axis_positions || [];

  const wearPct = toolWear ? Math.min(toolWear.wear_percentage || 0, 100) : null;
  const wearColor = wearPct === null ? 'inherit'
    : wearPct > 80 ? '#f44336'
    : wearPct > 50 ? '#ffa726'
    : '#66bb6a';

  // ------------------------------------------------------------------
  // Render
  // ------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>
      {/* Machine selector */}
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mb: 3 }}>
        <FormControl size="small" sx={{ minWidth: 180 }}>
          <InputLabel>Machine</InputLabel>
          <Select
            value={selectedMachine}
            label="Machine"
            onChange={(e) => setSelectedMachine(e.target.value)}
          >
            {DEFAULT_MACHINE_IDS.map((id) => (
              <MenuItem key={id} value={id}>{id}</MenuItem>
            ))}
          </Select>
        </FormControl>
        <Chip label={status} color={chipColor} />
      </Box>

      <Grid container spacing={2}>
        {/* State overview card */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Machine State</Typography>
              <Table size="small">
                <TableBody>
                  <TableRow>
                    <TableCell component="th">Spindle RPM</TableCell>
                    <TableCell align="right">
                      {machineState?.spindle_speed?.toFixed(0) ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Feed Rate (mm/min)</TableCell>
                    <TableCell align="right">
                      {machineState?.feed_rate?.toFixed(1) ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Spindle Load (%)</TableCell>
                    <TableCell align="right">
                      {machineState?.spindle_load?.toFixed(1) ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Coolant Level</TableCell>
                    <TableCell align="right">
                      {machineState?.coolant_level?.toFixed(0) ?? '--'}%
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Current Program</TableCell>
                    <TableCell align="right">
                      {machineState?.current_program || '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Line</TableCell>
                    <TableCell align="right">
                      {machineState?.current_line ?? '--'}
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Cycle Elapsed</TableCell>
                    <TableCell align="right">
                      {machineState?.cycle_time_elapsed?.toFixed(1) ?? '--'} s
                    </TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell component="th">Cycle Remaining</TableCell>
                    <TableCell align="right">
                      {machineState?.cycle_time_remaining?.toFixed(1) ?? '--'} s
                    </TableCell>
                  </TableRow>
                </TableBody>
              </Table>
            </CardContent>
          </Card>
        </Grid>

        {/* Axis positions */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Axis Positions</Typography>
              {axisPositions.length === 0 ? (
                <Typography variant="body2" color="text.secondary">
                  No axis data received yet.
                </Typography>
              ) : (
                axisPositions.map((pos, i) => (
                  <Box key={i} sx={{ mb: 1.5 }}>
                    <Box sx={{ display: 'flex', justifyContent: 'space-between' }}>
                      <Typography variant="body2" sx={{ fontWeight: 600 }}>
                        {axisLabels[i] || `Axis ${i}`}
                      </Typography>
                      <Typography variant="body2">
                        {pos.toFixed(3)} mm
                      </Typography>
                    </Box>
                    <LinearProgress
                      variant="determinate"
                      value={Math.min(Math.abs(pos) / 5 * 100, 100)} // normalise to 500mm travel
                      sx={{
                        height: 6,
                        borderRadius: 3,
                        bgcolor: 'rgba(255,255,255,0.06)',
                        '& .MuiLinearProgress-bar': { borderRadius: 3 },
                      }}
                    />
                  </Box>
                ))
              )}

              <Divider sx={{ my: 2 }} />

              {/* Tool wear */}
              <Typography variant="h6" gutterBottom>Tool Wear</Typography>
              {wearPct === null ? (
                <Typography variant="body2" color="text.secondary">
                  No tool wear data yet.
                </Typography>
              ) : (
                <Box>
                  <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 0.5 }}>
                    <Typography variant="body2">
                      {toolWear?.tool_id || 'Tool'} &mdash; {toolWear?.wear_type || ''}
                    </Typography>
                    <Typography variant="body2" sx={{ color: wearColor, fontWeight: 700 }}>
                      {wearPct.toFixed(1)}%
                    </Typography>
                  </Box>
                  <LinearProgress
                    variant="determinate"
                    value={wearPct}
                    sx={{
                      height: 10,
                      borderRadius: 5,
                      bgcolor: 'rgba(255,255,255,0.08)',
                      '& .MuiLinearProgress-bar': { bgcolor: wearColor, borderRadius: 5 },
                    }}
                  />
                  {toolWear?.remaining_life_minutes != null && (
                    <Typography variant="caption" color="text.secondary" sx={{ mt: 0.5, display: 'block' }}>
                      Est. remaining life: {toolWear.remaining_life_minutes.toFixed(0)} min
                    </Typography>
                  )}
                </Box>
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* Sensor charts */}
        <Grid item xs={12} md={4}>
          <Card sx={{ height: '100%' }}>
            <CardContent>
              <Typography variant="h6" gutterBottom>Sensor Data</Typography>
              {['vibration', 'current', 'temperature'].map((type) => {
                const data = sensorHistory[type];
                const colors = { vibration: '#f44336', current: '#29b6f6', temperature: '#ffa726' };
                const units  = { vibration: 'g', current: 'A', temperature: 'C' };
                return (
                  <Box key={type} sx={{ mb: 2 }}>
                    <Typography variant="subtitle2" sx={{ textTransform: 'capitalize', mb: 0.5 }}>
                      {type} ({units[type]})
                    </Typography>
                    {data.length < 2 ? (
                      <Typography variant="caption" color="text.secondary">
                        Waiting for data...
                      </Typography>
                    ) : (
                      <ResponsiveContainer width="100%" height={100}>
                        <LineChart data={data}>
                          <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.06)" />
                          <XAxis dataKey="time" tick={false} />
                          <YAxis width={40} tick={{ fontSize: 10 }} />
                          <Tooltip
                            contentStyle={{ background: '#1e1e3a', border: 'none', borderRadius: 8 }}
                          />
                          <Line
                            type="monotone"
                            dataKey="value"
                            stroke={colors[type]}
                            dot={false}
                            strokeWidth={2}
                            isAnimationActive={false}
                          />
                        </LineChart>
                      </ResponsiveContainer>
                    )}
                  </Box>
                );
              })}
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Box>
  );
}
