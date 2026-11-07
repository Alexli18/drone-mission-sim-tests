// Allure 3 report settings (the allure job in .github/workflows/sim-tests.yml).
export default {
  name: "drone-mission-sim-tests",
  plugins: {
    awesome: {
      // One index.html, so the downloaded artifact opens without a web server.
      options: { singleFile: true },
    },
  },
};
