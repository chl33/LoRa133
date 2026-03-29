<script>
  import { Save } from 'lucide-svelte';

  const API_BASE = '/api';

  export let lora;

  let loraConfig;
  let saving = false;
  let saveMessage = '';

  $: loraConfig = $lora;

  function updateField(field, value) {
    lora.update(l => {
      l[field] = value;
      return l;
    });
  }

  async function saveConfig() {
    saving = true;
    saveMessage = '';

    try {
      const response = await fetch(`${API_BASE}/lora`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(loraConfig)
      });

      if (!response.ok) throw new Error('Failed to save LoRa configuration');

      saveMessage = 'LoRa configuration saved successfully!';
      setTimeout(() => saveMessage = '', 3000);
    } catch (err) {
      saveMessage = `Error: ${err.message}`;
    } finally {
      saving = false;
    }
  }
</script>

<div class="page">
  <h2 class="page-title">LoRa Configuration</h2>

  <div class="card">
    <div class="form-grid">
      <div class="form-group">
        <label class="form-label">Frequency</label>
        <select 
          class="form-input" 
          bind:value={loraConfig.frequency}
          on:change={() => updateField('frequency', loraConfig.frequency)}>
          <option value="433MHz">433 MHz</option>
          <option value="868MHz">868 MHz</option>
          <option value="915MHz">915 MHz</option>
        </select>
      </div>

      <div class="form-group">
        <label class="form-label">Sync Word (HEX)</label>
        <div class="hex-input-wrapper">
          <span>0x</span>
          <input
            type="number"
            min="0"
            max="255"
            class="form-input"
            bind:value={loraConfig.sync_word}
            on:change={() => updateField('sync_word', loraConfig.sync_word)}
          />
        </div>
        <div class="form-hint">Default is 0x12 (Private) or 0x34 (Public)</div>
      </div>

      <div class="form-group">
        <label class="form-label">Spreading Factor</label>
        <select 
          class="form-input" 
          bind:value={loraConfig.spreading_factor}
          on:change={() => updateField('spreading_factor', loraConfig.spreading_factor)}>
          {#each ['SF7', 'SF8', 'SF9', 'SF10', 'SF11', 'SF12'] as sf}
            <option value={sf}>{sf}</option>
          {/each}
        </select>
      </div>

      <div class="form-group">
        <label class="form-label">Bandwidth</label>
        <select 
          class="form-input" 
          bind:value={loraConfig.signal_bandwidth}
          on:change={() => updateField('signal_bandwidth', loraConfig.signal_bandwidth)}>
          <option value="125k">125 kHz</option>
          <option value="500k">500 kHz</option>
        </select>
      </div>
    </div>

    <div class="form-group" style="margin-top: 1rem;">
      <label class="form-label">
        <input type="checkbox" bind:checked={loraConfig.enable_crc} />
        Enable hardware CRC
      </label>
    </div>

    <button class="btn btn-orange" on:click={saveConfig} disabled={saving}>
      <Save size={20} />
      {saving ? 'Saving...' : 'Save LoRa Configuration'}
    </button>

    {#if saveMessage}
      <div class="save-message" class:error={saveMessage.startsWith('Error')}>
        {saveMessage}
      </div>
    {/if}
  </div>
</div>

<style>
  .page-title {
    font-size: 2rem;
    font-weight: 700;
    color: #1f2937;
    margin-bottom: 1.5rem;
  }

  .card {
    background: white;
    padding: 1.5rem;
    border-radius: 0.5rem;
    box-shadow: 0 1px 3px rgba(0,0,0,0.1);
  }

  .form-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 1.5rem;
  }

  @media (max-width: 640px) {
    .form-grid {
      grid-template-columns: 1fr;
    }
  }

  .form-group {
    margin-bottom: 1rem;
  }

  .form-label {
    display: block;
    font-size: 0.875rem;
    font-weight: 500;
    color: #374151;
    margin-bottom: 0.5rem;
  }

  .form-input {
    width: 100%;
    padding: 0.5rem 1rem;
    border: 1px solid #d1d5db;
    border-radius: 0.5rem;
    font-size: 1rem;
    background: white;
  }

  .hex-input-wrapper {
    display: flex;
    align-items: center;
    gap: 0.5rem;
  }

  .form-hint {
    font-size: 0.75rem;
    color: #6b7280;
    margin-top: 0.25rem;
  }

  .btn {
    display: inline-flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.75rem 1.5rem;
    border: none;
    border-radius: 0.5rem;
    font-size: 1rem;
    font-weight: 500;
    cursor: pointer;
    transition: background 0.2s;
    margin-top: 1rem;
  }

  .btn-orange { background: #f59e0b; color: white; }
  .btn-orange:hover { background: #d97706; }
  .btn-orange:disabled { background: #9ca3af; }

  .save-message {
    margin-top: 1rem;
    padding: 0.75rem 1rem;
    border-radius: 0.5rem;
    background: #fef3c7;
    color: #92400e;
    font-size: 0.875rem;
    font-weight: 500;
  }

  .save-message.error {
    background: #fee2e2;
    color: #991b1b;
  }
</style>
