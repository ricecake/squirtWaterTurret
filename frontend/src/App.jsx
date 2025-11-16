import React, { useState } from 'react';
import { Tabs, Tab, Box } from '@mui/material';
import Visualizer from './components/Visualizer';
import UserClassification from './components/UserClassification';
import Logs from './components/Logs';
import Configuration from './components/Configuration';
import './App.css';

function TabPanel(props) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`simple-tabpanel-${index}`}
      aria-labelledby={`simple-tab-${index}`}
      {...other}
    >
      {value === index && (
        <Box sx={{ p: 3 }}>
          {children}
        </Box>
      )}
    </div>
  );
}

function a11yProps(index) {
  return {
    id: `simple-tab-${index}`,
    'aria-controls': `simple-tabpanel-${index}`,
  };
}

function App() {
  const [value, setValue] = useState(0);

  const handleChange = (event, newValue) => {
    setValue(newValue);
  };

  return (
    <Box sx={{ width: '100%' }}>
      <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
        <Tabs value={value} onChange={handleChange} aria-label="basic tabs example">
          <Tab label="Visualizer" {...a11yProps(0)} />
          <Tab label="User Classification" {...a11yProps(1)} />
          <Tab label="Logs" {...a11yProps(2)} />
          <Tab label="Configuration" {...a11yProps(3)} />
        </Tabs>
      </Box>
      <TabPanel value={value} index={0}>
        <Visualizer />
      </TabPanel>
      <TabPanel value={value} index={1}>
        <UserClassification />
      </TabPanel>
      <TabPanel value={value} index={2}>
        <Logs />
      </TabPanel>
      <TabPanel value={value} index={3}>
        <Configuration />
      </TabPanel>
    </Box>
  );
}

export default App;
