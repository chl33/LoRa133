<script>
  import { createEventDispatcher } from 'svelte';
  import { Wifi, Radio, Thermometer, Wind, Cpu, Settings } from 'lucide-svelte';

  export let devices;
  export let wifi;
  export let mqtt;
  export let systemStatus;

  const dispatch = createEventDispatcher();

  function navigate(page) {
    dispatch('changePage', page);
  }

  $: devicesList = $devices;
  $: wifiConfig = $wifi;
  $: mqttConfig = $mqtt;
  $: status = $systemStatus;
  $: activeDevices = devicesList.filter(d => !d.disabled).length;
</script>

<div class="page">
  <h2 class="page-title">System Overview</h2>

  <!-- System Status Bar -->
  <div class="system-status-bar">
    <div class="stat-compact">
      <span class="stat-icon-inline temp-color">
        <Thermometer size={18} />
      </span>
      <span class="stat-text">{(status.temperature || 0).toFixed(1)}°C</span>
    </div>

    <div class="stat-compact">
      <span class="stat-icon-inline humidity-color">
        <Wind size={18} />
      </span>
      <span class="stat-text">{(status.humidity || 0).toFixed(1)}%</span>
    </div>

    <div class="stat-compact">
      <span class="stat-icon-inline" class:status-ok={status.mqttConnected} class:status-error={!status.mqttConnected}>
        <Radio size={18} />
      </span>
      <span class="stat-text" class:status-ok={status.mqttConnected} class:status-error={!status.mqttConnected}>
        MQTT: {status.mqttConnected ? 'Connected' : 'Disconnected'}
      </span>
    </div>
  </div>

  <div class="card-grid">
    <!-- Devices Overview Card -->
    <div class="overview-card">
      <div class="card-header">
        <Cpu size={24} class="text-emerald-600" />
        <h3>Satellite Devices</h3>
        <button class="link-btn" on:click={() => navigate('devices')}>
          <Settings size={20} />
        </button>
      </div>
      <div class="device-stats">
        <div class="stat-item">
          <span class="stat-label">Total Registered</span>
          <span class="stat-value">{devicesList.length}</span>
        </div>
        <div class="stat-item">
          <span class="stat-label">Active (MQTT Enabled)</span>
          <span class="stat-value text-emerald-600">{activeDevices}</span>
        </div>
      </div>
      <button class="action-btn" on:click={() => navigate('devices')}>
        Manage Devices →
      </button>
    </div>

    <!-- LoRa Status Card -->
    <div class="status-card orange">
      <div class="card-header">
        <Radio size={24} />
        <h3>LoRa Radio</h3>
      </div>
      <div class="card-content">
        <p>Frequency: <strong>915 MHz</strong></p>
        <p>Ready to receive from satellite nodes.</p>
      </div>
      <button class="link-btn" on:click={() => navigate('lora')}>
        Configure Radio →
      </button>
    </div>
  </div>

  <div class="card-grid">
    <div class="status-card blue">
      <div class="card-header">
        <Wifi size={24} />
        <h3>WiFi Status</h3>
      </div>
      <p>{wifiConfig.essId || 'Not configured'}</p>
      <button class="link-btn" on:click={() => navigate('wifi')}>
        Configure WiFi →
      </button>
    </div>

    <div class="status-card purple">
      <div class="card-header">
        <Settings size={24} />
        <h3>MQTT Status</h3>
      </div>
      <p>{mqttConfig.hostAddr || 'Not configured'}</p>
      <button class="link-btn" on:click={() => navigate('mqtt')}>
        Configure MQTT →
      </button>
    </div>
  </div>
</div>

<style>
  .page-title {
    font-size: 2rem;
    font-weight: 700;
    color: #1f2937;
    margin-bottom: 1.5rem;
  }

  @media (max-width: 768px) {
    .page-title {
      font-size: 1.5rem;
      margin-bottom: 1rem;
    }
  }

  .card-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 1rem;
    margin-bottom: 1rem;
  }

  .overview-card {
    background: white;
    padding: 1.5rem;
    border-radius: 0.5rem;
    border: 1px solid #e5e7eb;
    display: flex;
    flex-direction: column;
  }

  .card-header {
    display: flex;
    align-items: center;
    gap: 0.75rem;
    margin-bottom: 1.5rem;
  }

  .card-header h3 {
    font-size: 1.25rem;
    font-weight: 600;
    flex: 1;
    margin: 0;
  }

  .device-stats {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1rem;
    margin-bottom: 1.5rem;
  }

  .stat-item {
    display: flex;
    flex-direction: column;
  }

  .stat-label {
    font-size: 0.75rem;
    color: #6b7280;
    text-transform: uppercase;
    font-weight: 600;
    margin-bottom: 0.25rem;
  }

  .stat-value {
    font-size: 1.5rem;
    font-weight: 700;
    color: #111827;
  }

  .status-card {
    padding: 1.5rem;
    border-radius: 0.5rem;
    border: 1px solid;
    display: flex;
    flex-direction: column;
  }

  .status-card.blue { background: #eff6ff; border-color: #bfdbfe; }
  .status-card.purple { background: #f5f3ff; border-color: #ddd6fe; }
  .status-card.orange { background: #fff7ed; border-color: #ffedd5; }

  .status-card p { color: #6b7280; margin-bottom: 0.5rem; }
  .status-card strong { color: #1f2937; }

  .link-btn {
    margin-top: auto;
    color: #3b82f6;
    font-weight: 500;
    background: none;
    border: none;
    cursor: pointer;
    font-size: 0.875rem;
    padding: 0;
    text-align: left;
  }

  .action-btn {
    margin-top: auto;
    background: #10b981;
    color: white;
    font-weight: 600;
    border: none;
    border-radius: 0.375rem;
    padding: 0.5rem 1rem;
    cursor: pointer;
    transition: background 0.2s;
  }

  .action-btn:hover { background: #059669; }

  .system-status-bar {
    display: flex;
    flex-wrap: wrap;
    gap: 1.5rem;
    padding: 1rem;
    background: white;
    border-radius: 0.5rem;
    border: 1px solid #e5e7eb;
    margin-bottom: 1.5rem;
  }

  .stat-compact {
    display: flex;
    align-items: center;
    gap: 0.5rem;
  }

  .temp-color { color: #dc2626; }
  .humidity-color { color: #2563eb; }
  .status-ok { color: #059669; }
  .status-error { color: #dc2626; }

  .stat-text {
    font-size: 0.875rem;
    font-weight: 600;
    color: #1f2937;
  }

  :global(.text-emerald-600) { color: #059669; }
</style>
