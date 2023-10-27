<script lang="ts">
  import { invoke } from "@tauri-apps/api/tauri";

  export let id: number;
  export let name: string;
  export let angle = 0;
  export let minAngle = -180;
  export let maxAngle = 180;
  export let minSpeed = 0;
  export let maxSpeed = 180;

  // Current and target values
  let targetSpeed = 0;
  let targetAngle = 0;

  // Input strings
  let angleInput: number | null = angle;
  let speedInput: number | null = targetSpeed;

  // Validity of input strings
  $: angleInputValid = angleInput !== null && angleInput >= minAngle && angleInput <= maxAngle;
  $: speedInputValid = speedInput !== null && speedInput >= minSpeed && speedInput <= maxSpeed;
  $: moving = targetAngle != angle;

  /**
   * Move the joint to the specified angle at the specified speed.
   *
   * @param angle The angle to move to
   * @param speed The speed to move at
   */
  async function moveTo(angle: number | null, speed: number | null) {
    if (angle === null || speed === null) return;

    console.log(`Moving joint ${id} to ${angle}°`);
    invoke("move_joint", { joint: id, angle, speed })
      .then(() => {
        targetAngle = angle;
        targetSpeed = speed;
      })
      .catch((e) => console.error(e));
  }

  /**
   * Stop the joint.
   */
  async function stop() {
    console.log(`Stopping joint ${id}`);
    invoke("stop_joint", { joint: id })
      .then(() => {
        console.log(`Joint ${id} stopped`);
        targetAngle = 0;
        targetSpeed = 0;
      })
      .catch((e) => console.error(e));
  }
</script>

<div id="main">
  <p id="joint-name">Joint {id + 1} ({name})</p>
  <div id="container">
    <div id="controls">
      <form
        on:submit={(e) => {
          e.preventDefault();
          if (angleInputValid && speedInputValid) moveTo(angleInput, speedInput);
        }}
      >
        <div id="controls-form">
          <div class="row">
            <p>Go to</p>
            <input
              type="number"
              id="angle-input"
              class={angleInputValid ? "" : "invalid"}
              bind:value={angleInput}
              min={minAngle}
              max={maxAngle}
              maxlength="4"
            />
            <p>°</p>
          </div>
          <div class="row">
            <p>at speed</p>
            <input
              type="number"
              id="angle-input"
              class={speedInputValid ? "" : "invalid"}
              bind:value={speedInput}
              min={minSpeed}
              max={maxSpeed}
            />
            <p>°/s</p>
          </div>
          <button
            type="button"
            disabled={!angleInputValid || !speedInputValid}
            on:click={() => moveTo(angleInput, speedInput)}>Submit</button
          >
        </div>
      </form>
    </div>
    <div id="info">
      <p style="text-align: end;">Angle:</p>
      <p>{angle ?? "N/A"}°</p>

      <p style="text-align: end;">Command angle:</p>
      <p>{targetAngle ?? "N/A"}°</p>

      <p style="text-align: end;">Command speed:</p>
      <p>{targetSpeed ?? "N/A"}°/s</p>
    </div>
  </div>
  <div id="stop-container">
    <button id="stop-btn" on:click={stop} disabled={!moving}>STOP</button>
  </div>
</div>

<style>
  p {
    margin: 4px;
    padding: 0;
    text-align: start;
  }

  input {
    width: 2em;
  }

  #main {
    display: flex;
    flex-direction: column;
    align-items: start;
    justify-content: center;

    box-sizing: border-box;

    width: 100%;
    min-width: 400px;
    max-width: 600px;

    border-radius: 16px;
    border: 1px solid #00000032;
    padding: 8px;
  }

  #container {
    display: flex;
    flex-direction: row;
    align-items: start;
    justify-content: space-between;

    width: 100%;
  }

  #info {
    display: grid;
    grid-template-columns: 1fr 0fr;
  }

  #controls {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;

    margin-top: 16px;
  }

  #controls-form {
    display: flex;
    flex-direction: column;
    align-items: start;
    justify-content: center;

    gap: 8px;
  }

  #stop-container {
    margin-top: 16px;
    width: 100%;
    display: flex;
    justify-content: end;
  }

  #stop-btn {
    width: 100%;
  }

  #stop-btn:enabled {
    color: white;
    background-color: red;
  }

  .row {
    display: flex;
    flex-direction: row;
    align-items: center;
    justify-content: start;
    gap: 8px;

    margin: 0;
    padding: 0;
  }

  .invalid {
    border: 1px solid #ff0000;
  }

  #joint-name {
    font-size: 0.8em;
    font-weight: bold;
    margin-bottom: 0;
  }

  @media (prefers-color-scheme: dark) {
    #main {
      background-color: #2f2f2f;
      box-shadow: 0 0 8px #00000064;
    }
  }
</style>
