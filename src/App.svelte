<script lang="ts">
  import { invoke } from "@tauri-apps/api/tauri";
  import JointControl from "./lib/JointControl.svelte";

  type JointInfo = {
    name: string;
    id: number;
  };

  const JOINTS: Array<JointInfo> = [
    { name: "base", id: 0 },
    { name: "shoulder", id: 1 },
    { name: "elbow", id: 2 },
    { name: "forearm roll", id: 3 },
    { name: "wrist pitch", id: 4 },
    { name: "wrist roll", id: 5 },
  ];

  let angles: Array<number> = Array(JOINTS.length).fill(0);
  let connected = false;
  let initialized = false;

  invoke("is_connected", {}).then((isConnected) => {
    connected = isConnected ? true : false;
    if (connected) {
      init();
    }
  });

  let portName = "/dev/ttyCobot0";
  let baudRate = 115200;

  /**
   * Initialize and calibrate the COBOT. Then, start listening for joint angle updates.
   */
  function init() {
    console.log("Initializing COBOT...");
    invoke("init", {})
      .then(() => {
        console.log("COBOT initialized");
        initialized = true;

        setInterval(() => {
          invoke("get_angles", {})
            .then((received) => {
              if (Array.isArray(received)) {
                received.forEach((angle, i) => {
                  angles[i] = angle;
                });
                angles = [...angles];
              }
            })
            .catch((e) => console.error(e));
        }, 100);
      })
      .catch((e) => {
        window.alert(e);
        invoke("disconnect", {}).then(() => (connected = false));
      });
  }
</script>

<main>
  <h1 id="title">COBOT Config Tester</h1>

  {#if connected}
    {#if initialized}
      <button on:click={() => invoke("calibrate", { joints: 0b111111 })}>Calibrate All</button>
      <div id="joints-container">
        {#each JOINTS as joint}
          <JointControl id={joint.id} name={joint.name} angle={angles[joint.id]} />
        {/each}
      </div>
    {:else}
      <p>Initializing...</p>
    {/if}
  {:else}
    <!-- Input serial port and baud rate -->
    <div id="input-area">
      <div class="input-container">
        <label for="port-name">Port name:</label>
        <input id="port-name" type="text" placeholder="Port name" bind:value={portName} />
      </div>
      <div class="input-container">
        <label for="baud-rate">Baud rate:</label>
        <input id="baud-rate" type="number" placeholder="Baud rate" bind:value={baudRate} />
      </div>
    </div>

    <!-- Connect button -->
    <div>
      <button
        on:click={() => {
          invoke("connect", { portName, baudRate })
            .then(() => {
              console.log("COBOT connected");
              connected = true;
              init();
            })
            .catch((e) => window.alert(e));
        }}
      >
        Connect
      </button>
    </div>
  {/if}
</main>

<style>
  main {
    margin: 0;
    padding: 0;
    display: flex;
    flex-direction: column;
    justify-content: center;
    text-align: center;
  }

  #title {
    margin: 32px;
  }

  #joints-container {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
    justify-items: center;
    grid-gap: 16px;

    box-sizing: border-box;
    margin: 16px;
  }

  #input-area {
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    margin: 32px;
  }

  .input-container {
    display: flex;
    flex-direction: row;
    margin: 8px;
    justify-content: center;
    align-items: center;
    gap: 8px;
  }
</style>
