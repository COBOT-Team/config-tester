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

  // Regularly poll the joints' angles and update the angles array
  setInterval(() => {
    invoke("get_position", {})
      .then((angles) => {
        if (Array.isArray(angles)) {
          angles.forEach((angle, i) => {
            angles[i] = angle;
          });
        }
      })
      .catch((e) => console.error(e));
  }, 20);
</script>

<main>
  <h1 id="title">COBOT Config Tester</h1>
  <div id="joints-container">
    {#each JOINTS as joint}
      <JointControl id={joint.id} name={joint.name} angle={angles[joint.id]} />
    {/each}
  </div>
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
</style>
