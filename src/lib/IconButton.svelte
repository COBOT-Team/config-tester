<script lang="ts">
  export let size = "32px";
  export let path: string;
  export let viewBox: string = "0 0 24 24";
  export let color: string | undefined = undefined;
  export let active = true;

  $: displayColor = active ? color : undefined;
</script>

<div class={active ? "active" : ""} id="icon-btn">
  <svg class={active ? "active" : ""} width={size} height={size} {viewBox}>
    <path fill={displayColor} d={path} />
  </svg>
</div>

<style>
  #icon-btn {
    display: flex;
    align-items: center;
    justify-content: center;

    border-radius: 50%;
    border: 1px solid transparent;
    padding: 4px;

    transition: all 0.1s ease-in-out;
  }

  #icon-btn.active:hover {
    background-color: #00000010;
    cursor: pointer;
  }

  #icon-btn.active:active {
    background-color: #00000032;
    border: 1px solid #00000032;
  }

  svg {
    width: 100%;
    height: 100%;

    opacity: 0.33;
    fill: #000000;
  }

  svg.active {
    opacity: 1;
  }

  @media (prefers-color-scheme: dark) {
    #icon-btn.active:hover {
      background-color: #ffffff32;
    }

    #icon-btn.active:active {
      background-color: #ffffff64;
      border: 1px solid #ffffff64;
    }
    svg {
      fill: #ffffff;
    }
  }
</style>
