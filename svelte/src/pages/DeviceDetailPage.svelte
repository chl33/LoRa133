<script>
  import { onMount, createEventDispatcher } from 'svelte';
  import { 
    Cpu, Trash2, ArrowLeft, RefreshCw, 
    Thermometer, Wind, Zap, Activity, 
    CheckCircle2, XCircle
  } from 'lucide-svelte';

  export let id;

  const dispatch = createEventDispatcher();
  const API_BASE = '/api';

  let device = null;
  let loading = true;
  let error = null;
  let saving = false;

  async function fetchDeviceDetails() {
    try {
      const response = await fetch(`${API_BASE}/device?id=${id}`);
      if (!response.ok) throw new Error('Device not found');
      device = await response.json();
      loading = false;
    } catch (err) {
      error = err.message;
      loading = false;
    }
  }

  async function toggleMqtt() {
    saving = true;
    try {
      const response = await fetch(`${API_BASE}/device/config?id=${id}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ disabled: !device.disabled })
      });
      if (!response.ok) throw new Error('Failed to update config');
      device.disabled = !device.disabled;
    } catch (err) {
      alert(err.message);
    } finally {
      saving = false;
    }
  }

  async function forgetDevice() {
    if (!confirm(`Are you sure you want to forget '${device.name}'? All history will be lost.`)) return;
    
    try {
      const response = await fetch(`${API_BASE}/device/forget?id=${id}`, {
        method: 'POST'
      });
      if (!response.ok) throw new Error('Failed to forget device');
      dispatch('changePage', 'devices');
    } catch (err) {
      alert(err.message);
    }
  }

  function getSensorIcon(type, name) {
    const n = name.toLowerCase();
    if (n.includes('temp')) return Thermometer;
    if (n.includes('hum')) return Wind;
    if (n.includes('volt') || n.includes('batt')) return Zap;
    return Activity;
  }

  onMount(() => {
    fetchDeviceDetails();
    const interval = setInterval(fetchDeviceDetails, 5000);
    return () => clearInterval(interval);
  });

  function goBack() {
    dispatch('changePage', 'devices');
  }
</script>

<div class="page">
  <div class="page-header">
    <button class="back-btn" on:click={goBack}>
      <ArrowLeft size={20} />
    </button>
    <h2 class="page-title">Device Details</h2>
  </div>

  {#if loading}
    <div class="loading-state">
      <RefreshCw size={32} class="spinner" />
      <p>Loading device details...</p>
    </div>
  {:else if error}
    <div class="error-state">
      <XCircle size={48} class="text-red-600" />
      <h3>Error Loading Device</h3>
      <p>{error}</p>
      <button class="btn btn-secondary" on:click={goBack}>Back to List</button>
    </div>
  {:else}
    <div class="detail-grid">
      <!-- Info Card -->
      <div class="card info-card">
        <div class="card-header">
          <Cpu size={24} />
          <div class="header-text">
            <h3>{device.name}</h3>
            <span>ID: 0x{device.id.toString(16).toUpperCase()}</span>
          </div>
        </div>
        
        <div class="info-rows">
          <div class="info-row">
            <span class="label">Type</span>
            <span class="value">{device.type}</span>
          </div>
          <div class="info-row">
            <span class="label">MQTT Updates</span>
            <span class="value" class:active={!device.disabled}>
              {!device.disabled ? 'Enabled' : 'Disabled'}
            </span>
          </div>
          <div class="info-row">
            <span class="label">Dropped Packets</span>
            <span class="value" class:warn={device.droppedPackets > 0}>
              {device.droppedPackets}
            </span>
          </div>
        </div>

        <div class="actions">
          <button 
            class="btn" 
            class:btn-success={device.disabled}
            class:btn-warning={!device.disabled}
            on:click={toggleMqtt} 
            disabled={saving}>
            {device.disabled ? 'Enable MQTT' : 'Disable MQTT'}
          </button>
          
          <button class="btn btn-danger" on:click={forgetDevice}>
            <Trash2 size={18} />
            <span>Forget Device</span>
          </button>
        </div>
      </div>

      <!-- Sensors Card -->
      <div class="card sensors-card">
        <div class="card-header">
          <Activity size={24} />
          <h3>Live Sensor Data</h3>
        </div>
        
        <div class="sensors-grid">
          {#each device.sensors as sensor}
            <div class="sensor-tile">
              <div class="sensor-icon">
                <svelte:component this={getSensorIcon(sensor.type, sensor.name)} size={20} />
              </div>
              <div class="sensor-info">
                <span class="sensor-name">{sensor.name}</span>
                <span class="sensor-value">
                  {sensor.value !== null ? sensor.value.toFixed(sensor.type === 'float' ? 2 : 0) : '--'}
                  <small>{sensor.units}</small>
                </span>
              </div>
            </div>
          {/each}
          {#if device.sensors.length === 0}
            <p class="no-sensors">No sensor data available for this device.</p>
          {/if}
        </div>
      </div>
    </div>
  {/if}
</div>

<style>
  .page-header {
    display: flex;
    align-items: center;
    gap: 1rem;
    margin-bottom: 2rem;
  }

  .back-btn {
    background: white;
    border: 1px solid #e5e7eb;
    border-radius: 0.5rem;
    padding: 0.5rem;
    cursor: pointer;
    color: #4b5563;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .back-btn:hover {
    background: #f9fafb;
    color: #111827;
  }

  .page-title {
    font-size: 2rem;
    font-weight: 700;
    color: #1f2937;
    margin: 0;
  }

  .detail-grid {
    display: grid;
    grid-template-columns: 1fr 2fr;
    gap: 1.5rem;
  }

  @media (max-width: 1024px) {
    .detail-grid {
      grid-template-columns: 1fr;
    }
  }

  .card {
    background: white;
    border-radius: 0.75rem;
    border: 1px solid #e5e7eb;
    padding: 1.5rem;
    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
  }

  .card-header {
    display: flex;
    align-items: center;
    gap: 1rem;
    margin-bottom: 1.5rem;
    color: #374151;
  }

  .card-header h3 {
    font-size: 1.25rem;
    font-weight: 600;
    margin: 0;
  }

  .header-text {
    display: flex;
    flex-direction: column;
  }

  .header-text span {
    font-size: 0.875rem;
    color: #6b7280;
  }

  .info-rows {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    margin-bottom: 2rem;
  }

  .info-row {
    display: flex;
    justify-content: space-between;
    padding-bottom: 0.75rem;
    border-bottom: 1px solid #f3f4f6;
  }

  .info-row .label {
    color: #6b7280;
    font-size: 0.875rem;
  }

  .info-row .value {
    font-weight: 600;
    color: #111827;
  }

  .info-row .value.active {
    color: #059669;
  }

  .info-row .value.warn {
    color: #dc2626;
  }

  .actions {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
  }

  .btn {
    width: 100%;
    padding: 0.75rem;
    border-radius: 0.5rem;
    font-weight: 600;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.5rem;
    transition: all 0.2s;
    border: none;
  }

  .btn-success { background: #10b981; color: white; }
  .btn-warning { background: #f59e0b; color: white; }
  .btn-danger { background: #fee2e2; color: #dc2626; border: 1px solid #fecaca; }
  .btn-danger:hover { background: #dc2626; color: white; }
  .btn-secondary { background: #f3f4f6; color: #374151; }

  .sensors-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
    gap: 1rem;
  }

  .sensor-tile {
    background: #f9fafb;
    border: 1px solid #f3f4f6;
    border-radius: 0.5rem;
    padding: 1rem;
    display: flex;
    align-items: center;
    gap: 1rem;
  }

  .sensor-icon {
    width: 40px;
    height: 40px;
    background: white;
    border-radius: 0.375rem;
    display: flex;
    align-items: center;
    justify-content: center;
    color: #3b82f6;
    box-shadow: 0 1px 2px rgba(0,0,0,0.05);
  }

  .sensor-info {
    display: flex;
    flex-direction: column;
  }

  .sensor-name {
    font-size: 0.75rem;
    color: #6b7280;
    text-transform: uppercase;
    font-weight: 600;
  }

  .sensor-value {
    font-size: 1.25rem;
    font-weight: 700;
    color: #111827;
  }

  .sensor-value small {
    font-size: 0.875rem;
    font-weight: 500;
    color: #6b7280;
  }

  .no-sensors {
    grid-column: 1 / -1;
    text-align: center;
    color: #9ca3af;
    padding: 2rem;
  }

  .spinner {
    animation: spin 1s linear infinite;
  }

  @keyframes spin {
    from { transform: rotate(0deg); }
    to { transform: rotate(360deg); }
  }

  .loading-state, .error-state {
    text-align: center;
    padding: 4rem;
  }
</style>
