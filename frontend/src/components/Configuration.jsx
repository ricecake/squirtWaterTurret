import React, { useState, useEffect } from 'react';
import { TextField, Button, Box, Paper, Typography } from '@mui/material';

const Configuration = () => {
  const [config, setConfig] = useState({});
  const [message, setMessage] = useState('');

  const fetchConfig = () => {
    fetch('http://localhost:5000/api/config')
      .then(response => response.json())
      .then(data => setConfig(data))
      .catch(error => console.error("Failed to fetch config:", error));
  };

  useEffect(() => {
    fetchConfig();
  }, []);

  const handleChange = (e) => {
    const { name, value } = e.target;
    // Handle numeric and boolean values correctly
    const isNumber = !isNaN(parseFloat(value)) && isFinite(value);
    const isBoolean = value.toLowerCase() === 'true' || value.toLowerCase() === 'false';

    setConfig(prevConfig => ({
      ...prevConfig,
      [name]: isBoolean ? (value.toLowerCase() === 'true') : (isNumber ? parseFloat(value) : value),
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    setMessage('');
    fetch('http://localhost:5000/api/config', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    })
    .then(response => response.json())
    .then(data => {
        setMessage(data.message || 'Configuration saved!');
        // Re-fetch config to show the saved values
        fetchConfig();
    })
    .catch(error => {
        setMessage('Failed to save configuration.');
        console.error("Failed to save config:", error);
    });
  };

  return (
    <Paper elevation={3} style={{ padding: '16px' }}>
      <Typography variant="h6" gutterBottom>
        System Configuration
      </Typography>
      <Box component="form" onSubmit={handleSubmit} sx={{ '& .MuiTextField-root': { m: 1, width: '95%' } }}>
        {Object.keys(config).map((key) => (
          <TextField
            key={key}
            label={key}
            name={key}
            value={config[key]}
            onChange={handleChange}
            variant="outlined"
            fullWidth
          />
        ))}
        <Button type="submit" variant="contained" color="primary" sx={{ m: 1 }}>
          Save Configuration
        </Button>
        {message && <Typography sx={{ m: 1, color: 'green' }}>{message}</Typography>}
      </Box>
    </Paper>
  );
};

export default Configuration;
