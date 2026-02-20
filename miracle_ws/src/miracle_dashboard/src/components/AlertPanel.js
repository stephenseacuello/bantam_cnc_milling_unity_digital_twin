import React, { useState, useEffect, useCallback } from 'react';
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
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Stack,
} from '@mui/material';
import CheckCircleOutlineIcon from '@mui/icons-material/CheckCircleOutline';

import { SECURITY_ALERTS, DEFAULT_MACHINE_IDS } from '../utils/rosTopics';

// ---------------------------------------------------------------------------
// Severity styling
// ---------------------------------------------------------------------------
const SEVERITY_CONFIG = {
  CRITICAL: { color: '#f44336', label: 'CRITICAL', chipColor: 'error' },
  HIGH:     { color: '#ff9800', label: 'HIGH',     chipColor: 'warning' },
  MEDIUM:   { color: '#fdd835', label: 'MEDIUM',   chipColor: 'default' },
  LOW:      { color: '#29b6f6', label: 'LOW',      chipColor: 'info' },
};

const getSeverityConfig = (sev) =>
  SEVERITY_CONFIG[(sev || '').toUpperCase()] || SEVERITY_CONFIG.LOW;

// ---------------------------------------------------------------------------
// AlertPanel component
// ---------------------------------------------------------------------------
export default function AlertPanel({ ros }) {
  const { subscribe, connected } = ros;

  const [alerts, setAlerts] = useState([]);
  const [acknowledged, setAcknowledged] = useState(new Set());

  // Filters
  const [filterSeverity, setFilterSeverity] = useState('ALL');
  const [filterMachine, setFilterMachine] = useState('ALL');
  const [filterType, setFilterType] = useState('ALL');

  // ------------------------------------------------------------------
  // Subscribe to alerts
  // ------------------------------------------------------------------
  const onAlert = useCallback((msg) => {
    setAlerts((prev) => [
      { ...msg, receivedAt: new Date().toISOString() },
      ...prev,
    ].slice(0, 200)); // keep last 200
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
  // Acknowledge handler
  // ------------------------------------------------------------------
  const handleAck = (alertId) => {
    setAcknowledged((prev) => new Set(prev).add(alertId));
  };

  // ------------------------------------------------------------------
  // Filtering
  // ------------------------------------------------------------------
  const uniqueTypes = [...new Set(alerts.map((a) => a.category).filter(Boolean))];

  const filteredAlerts = alerts.filter((a) => {
    if (filterSeverity !== 'ALL' && (a.severity || '').toUpperCase() !== filterSeverity) return false;
    if (filterMachine !== 'ALL' && !a.source_node?.includes(filterMachine)) return false;
    if (filterType !== 'ALL' && a.category !== filterType) return false;
    return true;
  });

  // ------------------------------------------------------------------
  // Render
  // ------------------------------------------------------------------
  return (
    <Box sx={{ p: 3 }}>
      {/* Filters */}
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>Filters</Typography>
          <Stack direction="row" spacing={2} flexWrap="wrap" useFlexGap>
            {/* Severity */}
            <FormControl size="small" sx={{ minWidth: 150 }}>
              <InputLabel>Severity</InputLabel>
              <Select
                value={filterSeverity}
                label="Severity"
                onChange={(e) => setFilterSeverity(e.target.value)}
              >
                <MenuItem value="ALL">All</MenuItem>
                {Object.keys(SEVERITY_CONFIG).map((s) => (
                  <MenuItem key={s} value={s}>{s}</MenuItem>
                ))}
              </Select>
            </FormControl>

            {/* Machine */}
            <FormControl size="small" sx={{ minWidth: 150 }}>
              <InputLabel>Machine</InputLabel>
              <Select
                value={filterMachine}
                label="Machine"
                onChange={(e) => setFilterMachine(e.target.value)}
              >
                <MenuItem value="ALL">All</MenuItem>
                {DEFAULT_MACHINE_IDS.map((id) => (
                  <MenuItem key={id} value={id}>{id}</MenuItem>
                ))}
              </Select>
            </FormControl>

            {/* Type / Category */}
            <FormControl size="small" sx={{ minWidth: 150 }}>
              <InputLabel>Type</InputLabel>
              <Select
                value={filterType}
                label="Type"
                onChange={(e) => setFilterType(e.target.value)}
              >
                <MenuItem value="ALL">All</MenuItem>
                {uniqueTypes.map((t) => (
                  <MenuItem key={t} value={t}>{t}</MenuItem>
                ))}
              </Select>
            </FormControl>

            <Chip
              label={`${filteredAlerts.length} alert${filteredAlerts.length !== 1 ? 's' : ''}`}
              variant="outlined"
              size="small"
              sx={{ alignSelf: 'center' }}
            />
          </Stack>
        </CardContent>
      </Card>

      {/* Alert table */}
      <Card>
        <CardContent sx={{ p: 0, '&:last-child': { pb: 0 } }}>
          <TableContainer sx={{ maxHeight: 'calc(100vh - 320px)' }}>
            <Table stickyHeader size="small">
              <TableHead>
                <TableRow>
                  <TableCell sx={{ fontWeight: 700 }}>Severity</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Source</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Category</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Description</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Recommendation</TableCell>
                  <TableCell sx={{ fontWeight: 700 }}>Confidence</TableCell>
                  <TableCell sx={{ fontWeight: 700 }} align="center">Action</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {filteredAlerts.length === 0 ? (
                  <TableRow>
                    <TableCell colSpan={7} align="center">
                      <Typography variant="body2" color="text.secondary" sx={{ py: 4 }}>
                        No alerts match the current filters.
                      </Typography>
                    </TableCell>
                  </TableRow>
                ) : (
                  filteredAlerts.map((alert, idx) => {
                    const cfg = getSeverityConfig(alert.severity);
                    const isAcked = acknowledged.has(alert.alert_id);
                    return (
                      <TableRow
                        key={alert.alert_id || idx}
                        sx={{
                          opacity: isAcked ? 0.5 : 1,
                          borderLeft: `4px solid ${cfg.color}`,
                        }}
                      >
                        <TableCell>
                          <Chip
                            label={cfg.label}
                            size="small"
                            sx={{
                              bgcolor: cfg.color,
                              color: '#000',
                              fontWeight: 700,
                              fontSize: '0.7rem',
                            }}
                          />
                        </TableCell>
                        <TableCell>{alert.source_node || '--'}</TableCell>
                        <TableCell>{alert.category || '--'}</TableCell>
                        <TableCell sx={{ maxWidth: 280, whiteSpace: 'normal' }}>
                          {alert.description || '--'}
                        </TableCell>
                        <TableCell sx={{ maxWidth: 200, whiteSpace: 'normal' }}>
                          {alert.recommended_action || '--'}
                        </TableCell>
                        <TableCell>
                          {alert.confidence != null
                            ? `${(alert.confidence * 100).toFixed(0)}%`
                            : '--'}
                        </TableCell>
                        <TableCell align="center">
                          {isAcked ? (
                            <Chip
                              icon={<CheckCircleOutlineIcon />}
                              label="ACK"
                              size="small"
                              color="success"
                              variant="outlined"
                            />
                          ) : (
                            <Button
                              size="small"
                              variant="outlined"
                              color="warning"
                              onClick={() => handleAck(alert.alert_id)}
                            >
                              Acknowledge
                            </Button>
                          )}
                        </TableCell>
                      </TableRow>
                    );
                  })
                )}
              </TableBody>
            </Table>
          </TableContainer>
        </CardContent>
      </Card>
    </Box>
  );
}
