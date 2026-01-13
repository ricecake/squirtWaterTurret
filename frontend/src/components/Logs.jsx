import React, { useState, useEffect } from 'react';
import { Paper, Typography } from '@mui/material';

const Logs = () => {
  const [logs, setLogs] = useState('');

  useEffect(() => {
    const fetchLogs = () => {
      fetch('http://localhost:5000/api/logs')
        .then(response => response.json())
        .then(data => setLogs(data.logs))
        .catch(error => console.error("Failed to fetch logs:", error));
    };

    fetchLogs(); // Initial fetch
    const interval = setInterval(fetchLogs, 5000); // Poll every 5 seconds

    return () => clearInterval(interval);
  }, []);

  return (
    <Paper elevation={3} style={{ padding: '16px', backgroundColor: '#f5f5f5' }}>
      <Typography variant="h6" gutterBottom>
        Application Logs
      </Typography>
      <pre style={{ whiteSpace: 'pre-wrap', wordWrap: 'break-word', margin: 0, fontFamily: 'monospace' }}>
        {logs || "No logs available."}
      </pre>
    </Paper>
  );
};

export default Logs;
