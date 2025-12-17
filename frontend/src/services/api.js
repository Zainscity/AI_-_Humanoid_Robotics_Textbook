import axios from 'axios';

const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export const getAuthConfig = (token) => ({
  headers: {
    Authorization: `Bearer ${token}`,
  },
});

export const login = async (username, password) => {
  const params = new URLSearchParams();
  params.append('username', username);
  params.append('password', password);
  const response = await axios.post(`${API_URL}/auth/token`, params);
  return response.data.access_token;
};

export const query = async (query, conversation_id, token) => {
  console.log('token in api.js:', token);
  const response = await axios.post(`${API_URL}/query`, { query, conversation_id }, getAuthConfig(token));
  return response.data;
};

export const selectedQuery = async (query, context, token) => {
  const response = await axios.post(`${API_URL}/query/selected-query`, { query, context }, getAuthConfig(token));
  return response.data;
};

export const getConversations = async (token) => {
  const response = await axios.get(`${API_URL}/history/conversations`, getAuthConfig(token));
  return response.data;
};

export const getMessages = async (conversationId, token) => {
  const response = await axios.get(`${API_URL}/history/conversations/${conversationId}/messages`, getAuthConfig(token));
  return response.data;
};

export const deleteConversation = async (conversationId, token) => {

  const response = await axios.delete(`${API_URL}/history/conversations/${conversationId}`, getAuthConfig(token));

  return response.data;

};



export const createConversation = async (token) => {

  const response = await axios.post(`${API_URL}/history/conversations`, {}, getAuthConfig(token));

  return response.data;

};
