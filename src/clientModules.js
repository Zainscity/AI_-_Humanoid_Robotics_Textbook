if (typeof process === 'undefined') {
  process = {};
}
if (typeof process.env === 'undefined') {
  process.env = {};
}

if (typeof window !== 'undefined' && window.Cypress) {
  // Expose env vars for Cypress tests
  process.env.REACT_APP_API_URL = window.Cypress.env('API_URL');
}
