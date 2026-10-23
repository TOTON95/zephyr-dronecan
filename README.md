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
- *Planned:* shell commands that decode and print message fields, and publish messages from field values (today they take message names and raw hex bytes)

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

A DroneCAN node is described in devicetree and attached to a CAN controller. The binding is
[`dts/bindings/zephyr,dronecan.yaml`](dts/bindings/zephyr,dronecan.yaml) and may change while the
`zephyr` vendor prefix is pending approval:

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
| `node-id` | Static node ID (1-127), or 0 for an anonymous node. |
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

### Messages

Every broadcast message type is a Kconfig option (`CONFIG_DRONECAN_MSG_<NAMESPACE>_<NAME>`), and
only the enabled ones are built. NodeStatus is always on, and the DNIA options select the
allocation message. After the DSDL revision changes, regenerate the options and the message
table, then commit the result:

```sh
python3 scripts/gen_kconfig_messages.py <DSDL directory>
```

### Shell

With `CONFIG_SHELL=y` and `CONFIG_DRONECAN_SHELL=y` the `dc` command is available. `<dev>` is the
devicetree device name and `<msg>` the full DSDL name of an enabled message.

| Command | Description |
| --- | --- |
| `dc list` | DroneCAN devices |
| `dc msgs` | Enabled messages |
| `dc status <dev>` | Node ID, counters, memory pool use and CAN controller state and error counters |
| `dc nodes <dev>` | Nodes seen on the bus (needs `CONFIG_DRONECAN_NODE_TABLE`) |
| `dc subscribe <dev> <msg>` | Print each received message as hex bytes |
| `dc unsubscribe <dev> <msg>` | Stop printing it |
| `dc monitor <dev> on\|off\|show` | Count the messages of every enabled type |
| `dc peripheral <dev> [timeout s]` | Ask an allocator for a node ID (the node needs `node-id = <0>`; needs `CONFIG_DRONECAN_DNIA_CLIENT`) |
| `dc publish <dev> <msg> <hex> [-r <hz>]` | Broadcast raw bytes, once or repeated at a rate |
| `dc unpublish <dev> <msg>` | Stop a repeated message |

## Samples

### `samples/dronecan`

A minimal application that declares a DroneCAN node in devicetree and calls the public API. It has
overlays for `native_sim` (the node sits on the CAN loopback controller) and for the 3DR Control N1.

```sh
west build -b native_sim samples/dronecan
west build -b control_n1 samples/dronecan
```

### `samples/dronecan_shell`

Starts a node and leaves the `dc` shell commands to drive it. On the 3DR Control N1 the shell runs
on a USB CDC ACM port and the console is turned off, so a USB cable is enough. On `native_sim` the
node sits on the CAN loopback controller. To try `dc peripheral`, set `node-id = <0>` in the
overlay and connect an allocator.

```sh
west build -b native_sim samples/dronecan_shell
west build -b control_n1 samples/dronecan_shell
```

### `samples/dronecan_peripheral`

An anonymous node (node ID 0) that asks an allocator on the bus for its node ID with
`dronecan_dnia_request()`, then reports its statistics. On the 3DR Control N1 the allocator is a
flight controller or the DroneCAN GUI Tool, or another board running `samples/dronecan` (it enables
the allocation server). On `native_sim` a second node plays the allocator.

```sh
west build -b native_sim samples/dronecan_peripheral
west build -b control_n1 samples/dronecan_peripheral
```

If the DSDL projects are not in the default location, point CMake at them with
`-DDRONECAN_DSDL_DIR=<path>` and `-DDRONECAN_DSDLC_DIR=<path>`.

## Tests

The tests are [ztest](https://docs.zephyrproject.org/latest/develop/test/ztest.html) suites that
run on `native_sim`, with the nodes on the CAN loopback controller:

```sh
west twister -T tests -p native_sim
```

| Suite | Covers |
| --- | --- |
| `tests/core` | Start and stop, publishing, subscriptions, multi-frame delivery, NodeStatus and the node table, statistics |
| `tests/dnia` | The allocation server and the allocation client |

Only one node on a bus should be an allocator. The allocation server option applies to the whole
image, so every node with an ID in the `tests/dnia` image answers requests: the suite keeps a single
node with an ID.

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

A firmware that includes these components must keep their copyright and permission notices: see
[THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
