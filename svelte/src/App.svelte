<script>
  import { writable } from 'svelte/store';
  import { onMount } from 'svelte';
  import Navbar from './components/Navbar.svelte';
  import Sidebar from './components/Sidebar.svelte';
  import HomePage from './pages/HomePage.svelte';
  import DevicesPage from './pages/DevicesPage.svelte';
  import DeviceDetailPage from './pages/DeviceDetailPage.svelte';
  import LoraConfigPage from './pages/LoraConfigPage.svelte';
  import WiFiConfigPage from './pages/WiFiConfigPage.svelte';
  import MQTTConfigPage from './pages/MQTTConfigPage.svelte';

  // API base URL
  const API_BASE = '/api';

  let currentPage = 'home';
  let detailDeviceId = null;
  let loading = true;
  let error = null;

  export let devices = writable([]);

  export let lora = writable({
    sync_word: 0xF0,
    spreading_factor: 'SF8',
    signal_bandwidth: '125k',
    frequency: '915MHz'
  });

  export let wifi = writable({
    essId: '',
    wifiPassword: '',
    board: 'lora133'
  });

  export let mqtt = writable({
    hostAddr: '',
    port: 1883,
    authUser: '',
    authPassword: '',
  });

  export let systemStatus = writable({
    temperature: 0,
    humidity: 0,
    mqttConnected: false,
    software: '',
    hardware: ''
  });

  export let isOnline = writable(true);

  async function fetchDevices() {
    try {
      const response = await fetch(`${API_BASE}/devices`);
      if (!response.ok) throw new Error('Failed to load devices');
      const data = await response.json();
      devices.set(data);
    } catch (err) {
      console.error('Error fetching devices:', err);
    }
  }

  async function fetchStatus() {
    try {
      const response = await fetch(`${API_BASE}/status`);
      if (!response.ok) throw new Error('Failed to load status');
      const data = await response.json();
      systemStatus.update(s => ({ ...s, ...data }));
      isOnline.set(true);
    } catch (err) {
      console.error('Error fetching status:', err);
      isOnline.set(false);
    }
  }

  async function fetchWifi() {
    try {
      const response = await fetch(`${API_BASE}/wifi`);
      if (!response.ok) throw new Error('Failed to load WiFi config');
      const data = await response.json();
      wifi.set(data);
    } catch (err) {
      console.error('Error fetching WiFi:', err);
    }
  }

  async function fetchMqtt() {
    try {
      const response = await fetch(`${API_BASE}/mqtt`);
      if (!response.ok) throw new Error('Failed to load MQTT config');
      const data = await response.json();
      mqtt.set(data);
    } catch (err) {
      console.error('Error fetching MQTT:', err);
    }
  }

  async function fetchLora() {
    try {
      const response = await fetch(`${API_BASE}/lora`);
      if (!response.ok) throw new Error('Failed to load LoRa config');
      const data = await response.json();
      lora.set(data);
    } catch (err) {
      console.error('Error fetching LoRa:', err);
    }
  }

  onMount(async () => {
    try {
      await Promise.all([
        fetchStatus(),
        fetchWifi(),
        fetchMqtt(),
        fetchLora(),
        fetchDevices()
      ]);
      loading = false;
    } catch (err) {
      error = err.message;
      loading = false;
    }

    const interval = setInterval(() => {
      fetchStatus();
      fetchDevices();
    }, 5000);

    return () => clearInterval(interval);
  });

  function handlePageChange(event) {
    const page = event.detail;
    if (page.startsWith('device-')) {
      detailDeviceId = page.split('-')[1];
      currentPage = 'device-detail';
    } else {
      currentPage = page;
      detailDeviceId = null;
    }
  }
</script>

<div class="app-container">
  <Sidebar {currentPage} {devices} {systemStatus} on:changePage={handlePageChange} />
  
  <main class="main-content">
    <Navbar {wifi} {isOnline} />
    
    <div class="content-wrapper">
      {#if loading}
        <div class="loading-state">
          <div class="spinner"></div>
          <p>Connecting to LoRa133...</p>
        </div>
      {:else if error}
        <div class="error-state">
          <p class="error-message">Error: {error}</p>
          <p class="error-hint">Please check your connection and try again.</p>
        </div>
      {:else}
        {#if currentPage === 'home'}
          <HomePage {devices} {wifi} {mqtt} {systemStatus} on:changePage={handlePageChange} />
        {:else if currentPage === 'devices'}
          <DevicesPage {devices} on:changePage={handlePageChange} />
        {:else if currentPage === 'device-detail'}
          <DeviceDetailPage id={detailDeviceId} on:changePage={handlePageChange} />
        {:else if currentPage === 'lora'}
          <LoraConfigPage {lora} />
        {:else if currentPage === 'wifi'}
          <WiFiConfigPage {wifi} />
        {:else if currentPage === 'mqtt'}
          <MQTTConfigPage {mqtt} {systemStatus} />
        {/if}
      {/if}
    </div>
  </main>
</div>

<style>
  :global(body) {
    margin: 0;
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    background-color: #f3f4f6;
    color: #1f2937;
  }

  .app-container {
    display: flex;
    min-height: 100vh;
  }

  @media (max-width: 768px) {
    .app-container {
      flex-direction: column;
    }
  }

  .main-content {
    flex: 1;
    display: flex;
    flex-direction: column;
    min-width: 0;
  }

  .content-wrapper {
    flex: 1;
    padding: 2rem;
    max-width: 1200px;
    margin: 0 auto;
    width: 100%;
    box-sizing: border-box;
  }

  @media (max-width: 768px) {
    .content-wrapper {
      padding: 1rem;
    }
  }

  .loading-state, .error-state {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    height: 60vh;
    text-align: center;
  }

  .spinner {
    width: 40px;
    height: 40px;
    border: 4px solid #e5e7eb;
    border-top: 4px solid #10b981;
    border-radius: 50%;
    animation: spin 1s linear infinite;
    margin-bottom: 1rem;
  }

  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }

  .error-message {
    color: #991b1b;
    font-weight: 600;
    margin-bottom: 0.5rem;
  }

  .error-hint {
    color: #6b7280;
    font-size: 0.875rem;
  }
</style>
