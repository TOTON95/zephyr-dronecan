# DroneCAN Protocol Stack for Zephyr RTOS

A [DroneCAN](https://dronecan.github.io/) (UAVCAN v0) protocol stack for [Zephyr RTOS](https://zephyrproject.org/).

Built on [libcanard](https://github.com/dronecan/libcanard/), with devicetree-configured, multi-instanced nodes that sit on top of Zephyr's CAN controller API.

> **Status:** early development. APIs may change before the first tagged release.

## Features

- Devicetree-configured nodes: node ID, bitrate and memory pool come from the overlay
- Multiple instances (for example, one node per CAN bus)
- NodeStatus broadcast and node discovery table
- Dynamic Node ID Allocation (DNIA): server and client
- Callback-based message and service subscription
- Kconfig-selectable message set (so unused DSDL code is not built)
- Optional shell commands (`CONFIG_DRONECAN_SHELL`): list nodes, show stats, monitor and subscribe to messages, publish test messages
- *Planned:* GetNodeInfo, parameter get/set, RestartNode, firmware update

## Requirements

- Zephyr (version/fork: *TBD*)
- A board with a Zephyr CAN controller driver and a CAN transceiver
- Python 3 and the packages in [`requirements.txt`](requirements.txt) (for DSDL code generation at build time)

## Being tested on

- 3DR Control N1 (STM32H743)
- 3DR KitCAN (STM32F303)

## Getting started

### Add the module to your west manifest

libcanard is included as a git submodule, so the project entry must set `submodules: true`:

```yaml
manifest:
  projects:
    - name: zephyr-dronecan
      url: <repo url>
      revision: main
      path: modules/lib/zephyr-dronecan
      submodules: true
```

The DSDL definitions and the DSDL code generator (`dronecan_dsdlc`) are west projects. Your manifest
must provide them too, either by importing this repository's [`west.yml`](west.yml) or by listing
`DSDL` and `dronecan_dsdlc` itself with the revisions pinned there.

Then run `west update`. If you clone the repository directly, use `git clone --recursive`
(or `git submodule update --init` afterwards).

### Install the Python dependencies

The module declares [`requirements.txt`](requirements.txt) for `west packages`, so this installs
them together with Zephyr's own requirements:

```sh
west packages pip --install
```

or install only this module's packages with `pip install -r requirements.txt`.

## Configuration

### Devicetree

A DroneCAN node is described in devicetree and attached to a CAN controller. This is the proposed
definition (the binding is not in the repository yet and may change):

```dts
dronecan0: dronecan {
	compatible = "zephyr,dronecan";
	can-buses = <&fdcan1>;
	node-id = <42>;
	mem-blocks = <64>;
	status = "okay";
};
```

| Property | Description |
| --- | --- |
| `can-buses` | CAN controller used by the node. One controller is supported for now. |
| `node-id` | Static node ID (1-127), or 0 to stay anonymous until an ID is allocated. |
| `mem-blocks` | libcanard memory pool size in blocks (default 64). |

The bus speed is not a property of the DroneCAN node. It is set on the CAN controller with the
standard `bitrate` property, and DroneCAN normally uses 1 Mbit/s.

#### 3DR Control N1

The board already enables `fdcan1` (pins and clocks are set in the board devicetree). Set the
bitrate in your overlay and add the node:

```dts
&fdcan1 {
	bitrate = <1000000>;
};

/ {
	dronecan0: dronecan {
		compatible = "zephyr,dronecan";
		can-buses = <&fdcan1>;
		node-id = <42>;
		status = "okay";
	};
};
```

Then enable the module in `prj.conf`:

```ini
CONFIG_CAN=y
CONFIG_DRONECAN=y
```

## Samples

## Tests

## Repository layout

## Compatibility

- Interoperability target: ArduPilot AP\_Periph and PX4 DroneCAN peripherals, DroneCAN GUI Tool
- Devicetree compatible: `zephyr,dronecan`. This module aims to be vendor neutral, so it uses the `zephyr` vendor prefix. That is a proposal for upstream and is pending approval by the Zephyr maintainers, so it may change.
- Zephyr version support is *TBD*

## Contributing

Contributions are welcome. Please follow Zephyr's coding style, add tests, and sign off your commits (`git commit -s`) under the [Developer Certificate of Origin](https://developercertificate.org/).

## License

Apache-2.0. See [LICENSE](LICENSE).

Third-party components keep their own licenses:
- libcanard: MIT
- DroneCAN DSDL definitions: MIT
- dronecan_dsdlc (build-time generator, not distributed): (L)GPL v3; generated code is MIT
