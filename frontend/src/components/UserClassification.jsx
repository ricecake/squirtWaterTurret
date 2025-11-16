import React, { useState, useEffect } from 'react';
import { Table, TableBody, TableCell, TableContainer, TableHead, TableRow, Paper, TextField, Button, Typography } from '@mui/material';

const UserClassification = () => {
  const [users, setUsers] = useState([]);
  const [editingId, setEditingId] = useState(null);
  const [editFormData, setEditFormData] = useState({ name: '', type: '' });

  const fetchUsers = () => {
    fetch('http://localhost:5000/api/users')
      .then(response => response.json())
      .then(data => setUsers(data));
  };

  useEffect(() => {
    fetchUsers();
  }, []);

  const handleEditClick = (user) => {
    setEditingId(user.id);
    setEditFormData({ name: user.name, type: user.type });
  };

  const handleCancelClick = () => {
    setEditingId(null);
  };

  const handleFormChange = (e) => {
    setEditFormData({ ...editFormData, [e.target.name]: e.target.value });
  };

  const handleFormSubmit = (e) => {
    e.preventDefault();
    fetch(`http://localhost:5000/api/users/${editingId}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(editFormData),
    }).then(() => {
      fetchUsers();
      setEditingId(null);
    });
  };

  return (
    <Paper>
      <Typography variant="h6" component="h1" style={{ padding: '16px' }}>
        User Classification
      </Typography>
      <TableContainer>
        <Table>
          <TableHead>
            <TableRow>
              <TableCell>ID</TableCell>
              <TableCell>Name</TableCell>
              <TableCell>Type</TableCell>
              <TableCell>Actions</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {users.map((user) => (
              <TableRow key={user.id}>
                <TableCell>{user.id}</TableCell>
                {editingId === user.id ? (
                  <>
                    <TableCell>
                      <TextField name="name" value={editFormData.name} onChange={handleFormChange} />
                    </TableCell>
                    <TableCell>
                      <TextField name="type" value={editFormData.type} onChange={handleFormChange} />
                    </TableCell>
                    <TableCell>
                      <Button onClick={handleFormSubmit}>Save</Button>
                      <Button onClick={handleCancelClick}>Cancel</Button>
                    </TableCell>
                  </>
                ) : (
                  <>
                    <TableCell>{user.name}</TableCell>
                    <TableCell>{user.type}</TableCell>
                    <TableCell>
                      <Button onClick={() => handleEditClick(user)}>Edit</Button>
                    </TableCell>
                  </>
                )}
              </TableRow>
            ))}
          </TableBody>
        </Table>
      </TableContainer>
    </Paper>
  );
};

export default UserClassification;
