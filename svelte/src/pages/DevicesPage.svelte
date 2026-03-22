<script>
  import { createEventDispatcher } from 'svelte';
  import { Cpu, Signal, AlertTriangle, ChevronRight } from 'lucide-svelte';

  export let devices;

  const dispatch = createEventDispatcher();

  function navigate(page) {
    dispatch('changePage', page);
  }

  $: devicesList = $devices;
</script>

<div class="page">
  <div class="page-header">
    <h2 class="page-title">Registered Devices</h2>
    <div class="badge">{devicesList.length} Total</div>
  </div>

  {#if devicesList.length === 0}
    <div class="empty-state">
      <Cpu size={48} class="text-gray-400" />
      <h3>No devices registered yet</h3>
      <p>Satellite devices will appear here once they transmit their first LoRa packet.</p>
    </div>
  {:else}
    <div class="card">
      <div class="table-container">
        <table class="device-table">
          <thead>
            <tr>
              <th>Device Name</th>
              <th>Type</th>
              <th>Status</th>
              <th>Dropped</th>
              <th class="text-right">Action</th>
            </tr>
          </thead>
          <tbody>
            {#each devicesList as device}
              <tr class="device-row" on:click={() => navigate(`device-${device.id}`)}>
                <td>
                  <div class="device-name-cell">
                    <span class="device-icon">
                      <Cpu size={18} />
                    </span>
                    <div class="name-info">
                      <span class="name">{device.name}</span>
                      <span class="id">ID: 0x{device.id.toString(16).toUpperCase()}</span>
                    </div>
                  </div>
                </td>
                <td>{device.type}</td>
                <td>
                  <span class="status-badge" class:active={!device.disabled}>
                    {!device.disabled ? 'Active' : 'MQTT Disabled'}
                  </span>
                </td>
                <td>
                  <div class="dropped-cell" class:has-errors={device.droppedPackets > 0}>
                    {#if device.droppedPackets > 0}
                      <AlertTriangle size={14} />
                    {/if}
                    {device.droppedPackets}
                  </div>
                </td>
                <td class="text-right">
                  <button class="manage-btn">
                    <span>Manage</span>
                    <ChevronRight size={16} />
                  </button>
                </td>
              </tr>
            {/each}
          </tbody>
        </table>
      </div>
    </div>
  {/if}
</div>

<style>
  .page-header {
    display: flex;
    align-items: center;
    gap: 1rem;
    margin-bottom: 1.5rem;
  }

  .page-title {
    font-size: 2rem;
    font-weight: 700;
    color: #1f2937;
    margin: 0;
  }

  .badge {
    background: #e5e7eb;
    color: #4b5563;
    padding: 0.25rem 0.75rem;
    border-radius: 9999px;
    font-size: 0.875rem;
    font-weight: 600;
  }

  .card {
    background: white;
    border-radius: 0.5rem;
    border: 1px solid #e5e7eb;
    overflow: hidden;
  }

  .table-container {
    overflow-x: auto;
  }

  .device-table {
    width: 100%;
    border-collapse: collapse;
    text-align: left;
  }

  th {
    background: #f9fafb;
    padding: 1rem;
    font-size: 0.75rem;
    font-weight: 600;
    color: #6b7280;
    text-transform: uppercase;
    border-bottom: 1px solid #e5e7eb;
  }

  td {
    padding: 1rem;
    border-bottom: 1px solid #e5e7eb;
    color: #374151;
  }

  .device-row {
    cursor: pointer;
    transition: background 0.2s;
  }

  .device-row:hover {
    background: #f9fafb;
  }

  .device-name-cell {
    display: flex;
    align-items: center;
    gap: 0.75rem;
  }

  .device-icon {
    width: 32px;
    height: 32px;
    background: #f3f4f6;
    color: #6b7280;
    border-radius: 0.375rem;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .name-info {
    display: flex;
    flex-direction: column;
  }

  .name {
    font-weight: 600;
    color: #111827;
  }

  .id {
    font-size: 0.75rem;
    color: #9ca3af;
  }

  .status-badge {
    display: inline-flex;
    padding: 0.25rem 0.75rem;
    border-radius: 9999px;
    font-size: 0.75rem;
    font-weight: 600;
    background: #fee2e2;
    color: #991b1b;
  }

  .status-badge.active {
    background: #d1fae5;
    color: #065f46;
  }

  .dropped-cell {
    display: flex;
    align-items: center;
    gap: 0.25rem;
    font-size: 0.875rem;
  }

  .has-errors {
    color: #dc2626;
    font-weight: 600;
  }

  .text-right {
    text-align: right;
  }

  .manage-btn {
    display: inline-flex;
    align-items: center;
    gap: 0.25rem;
    background: transparent;
    border: none;
    color: #3b82f6;
    font-weight: 600;
    font-size: 0.875rem;
    cursor: pointer;
  }

  .empty-state {
    text-align: center;
    padding: 4rem 2rem;
    background: white;
    border-radius: 0.5rem;
    border: 1px solid #e5e7eb;
  }

  .empty-state h3 {
    font-size: 1.25rem;
    font-weight: 600;
    margin: 1rem 0 0.5rem;
  }

  .empty-state p {
    color: #6b7280;
  }

  :global(.text-gray-400) { color: #9ca3af; }
</style>
