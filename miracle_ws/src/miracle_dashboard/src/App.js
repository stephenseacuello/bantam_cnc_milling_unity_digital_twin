import React, { useState, useMemo } from 'react';
import {
  ThemeProvider,
  createTheme,
  CssBaseline,
  Box,
  Drawer,
  AppBar,
  Toolbar,
  Typography,
  List,
  ListItemButton,
  ListItemIcon,
  ListItemText,
  Divider,
  Chip,
} from '@mui/material';
import DashboardIcon from '@mui/icons-material/Dashboard';
import PrecisionManufacturingIcon from '@mui/icons-material/PrecisionManufacturing';
import NotificationsActiveIcon from '@mui/icons-material/NotificationsActive';
import ViewInArIcon from '@mui/icons-material/ViewInAr';
import PsychologyIcon from '@mui/icons-material/Psychology';
import SecurityIcon from '@mui/icons-material/Security';
import SpeedIcon from '@mui/icons-material/Speed';
import FiberManualRecordIcon from '@mui/icons-material/FiberManualRecord';

import useROS from './hooks/useROS';
import Dashboard from './components/Dashboard';
import MachineView from './components/MachineView';
import AlertPanel from './components/AlertPanel';
import OEEPanel from './components/OEEPanel';
import DigitalTwinView from './components/DigitalTwinView';
import AIMLView from './components/AIMLView';
import SecurityView from './components/SecurityView';

const DRAWER_WIDTH = 240;

// ---------------------------------------------------------------------------
// Navigation items
// ---------------------------------------------------------------------------
const NAV_ITEMS = [
  { key: 'overview',     label: 'Overview',     icon: <DashboardIcon /> },
  { key: 'machines',     label: 'Machines',     icon: <PrecisionManufacturingIcon /> },
  { key: 'alerts',       label: 'Alerts',       icon: <NotificationsActiveIcon /> },
  { key: 'digital-twin', label: 'Digital Twin', icon: <ViewInArIcon /> },
  { key: 'ai-ml',        label: 'AI / ML',      icon: <PsychologyIcon /> },
  { key: 'security',     label: 'Security',     icon: <SecurityIcon /> },
  { key: 'oee',          label: 'OEE',          icon: <SpeedIcon /> },
];

// ---------------------------------------------------------------------------
// Dark manufacturing theme
// ---------------------------------------------------------------------------
const darkTheme = createTheme({
  palette: {
    mode: 'dark',
    primary:    { main: '#4fc3f7' },
    secondary:  { main: '#ff9800' },
    error:      { main: '#f44336' },
    warning:    { main: '#ffa726' },
    info:       { main: '#29b6f6' },
    success:    { main: '#66bb6a' },
    background: {
      default: '#0a0a1a',
      paper:   '#121228',
    },
  },
  typography: {
    fontFamily: '"Roboto", "Helvetica", "Arial", sans-serif',
    h4: { fontWeight: 700 },
    h5: { fontWeight: 600 },
    h6: { fontWeight: 600 },
  },
  components: {
    MuiCard: {
      styleOverrides: {
        root: {
          backgroundImage: 'none',
          borderRadius: 12,
          border: '1px solid rgba(255,255,255,0.06)',
        },
      },
    },
    MuiPaper: {
      styleOverrides: {
        root: {
          backgroundImage: 'none',
        },
      },
    },
  },
});

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------
export default function App() {
  const [activeView, setActiveView] = useState('overview');
  const { connected, error, subscribe, callService } = useROS();

  // Memoised ROS context object passed to child views
  const rosCtx = useMemo(
    () => ({ connected, error, subscribe, callService }),
    [connected, error, subscribe, callService],
  );

  // -----------------------------------------------------------------------
  // Render the active view
  // -----------------------------------------------------------------------
  const renderView = () => {
    switch (activeView) {
      case 'overview':
        return <Dashboard ros={rosCtx} />;
      case 'machines':
        return <MachineView ros={rosCtx} />;
      case 'alerts':
        return <AlertPanel ros={rosCtx} />;
      case 'oee':
        return <OEEPanel ros={rosCtx} />;
      case 'digital-twin':
        return <DigitalTwinView ros={rosCtx} />;
      case 'ai-ml':
        return <AIMLView ros={rosCtx} />;
      case 'security':
        return <SecurityView ros={rosCtx} />;
      default:
        return null;
    }
  };

  return (
    <ThemeProvider theme={darkTheme}>
      <CssBaseline />
      <Box sx={{ display: 'flex', minHeight: '100vh' }}>
        {/* ---- Sidebar ------------------------------------------------- */}
        <Drawer
          variant="permanent"
          sx={{
            width: DRAWER_WIDTH,
            flexShrink: 0,
            '& .MuiDrawer-paper': {
              width: DRAWER_WIDTH,
              boxSizing: 'border-box',
              bgcolor: 'background.paper',
              borderRight: '1px solid rgba(255,255,255,0.06)',
            },
          }}
        >
          <Toolbar sx={{ justifyContent: 'center', py: 2 }}>
            <Typography variant="h6" sx={{ fontWeight: 700, letterSpacing: 2 }}>
              MIRACLE
            </Typography>
          </Toolbar>
          <Divider />
          <List>
            {NAV_ITEMS.map((item) => (
              <ListItemButton
                key={item.key}
                selected={activeView === item.key}
                onClick={() => setActiveView(item.key)}
                sx={{
                  '&.Mui-selected': {
                    bgcolor: 'rgba(79,195,247,0.12)',
                    borderRight: '3px solid',
                    borderColor: 'primary.main',
                  },
                }}
              >
                <ListItemIcon sx={{ color: activeView === item.key ? 'primary.main' : 'inherit' }}>
                  {item.icon}
                </ListItemIcon>
                <ListItemText primary={item.label} />
              </ListItemButton>
            ))}
          </List>
        </Drawer>

        {/* ---- Main content -------------------------------------------- */}
        <Box component="main" sx={{ flexGrow: 1, display: 'flex', flexDirection: 'column' }}>
          <AppBar
            position="static"
            elevation={0}
            sx={{ bgcolor: 'background.paper', borderBottom: '1px solid rgba(255,255,255,0.06)' }}
          >
            <Toolbar>
              <Typography variant="h6" sx={{ flexGrow: 1 }}>
                {NAV_ITEMS.find((n) => n.key === activeView)?.label || ''}
              </Typography>
              <Chip
                icon={
                  <FiberManualRecordIcon
                    sx={{ fontSize: 12, color: connected ? 'success.main' : 'error.main' }}
                  />
                }
                label={connected ? 'ROS Connected' : 'ROS Disconnected'}
                variant="outlined"
                size="small"
                sx={{ borderColor: connected ? 'success.main' : 'error.main' }}
              />
            </Toolbar>
          </AppBar>

          <Box sx={{ flexGrow: 1, overflow: 'auto', bgcolor: 'background.default' }}>
            {renderView()}
          </Box>
        </Box>
      </Box>
    </ThemeProvider>
  );
}
