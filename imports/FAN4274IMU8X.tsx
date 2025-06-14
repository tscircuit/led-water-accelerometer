import type { ChipProps } from "@tscircuit/props"

const pinLabels = {
  pin1: ["pin1"],
  pin2: ["pin2"],
  pin3: ["pin3"],
  pin4: ["pin4"],
  pin5: ["pin5"],
  pin6: ["pin6"],
  pin7: ["pin7"],
  pin8: ["pin8"]
} as const

export const FAN4274IMU8X = (props: ChipProps<typeof pinLabels>) => {
  return (
    <chip
      pinLabels={pinLabels}
      supplierPartNumbers={{
  "jlcpcb": [
    "C124467"
  ]
}}
      manufacturerPartNumber="FAN4274IMU8X"
      footprint={<footprint>
        <smtpad portHints={["pin1"]} pcbX="-0.9751059999999967mm" pcbY="-2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin2"]} pcbX="-0.3251199999999983mm" pcbY="-2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin3"]} pcbX="0.3251200000000125mm" pcbY="-2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin4"]} pcbX="0.9751059999999967mm" pcbY="-2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin5"]} pcbX="0.9751059999999967mm" pcbY="2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin6"]} pcbX="0.3251200000000125mm" pcbY="2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin7"]} pcbX="-0.3251199999999983mm" pcbY="2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<smtpad portHints={["pin8"]} pcbX="-0.9751059999999967mm" pcbY="2.299970000000002mm" width="0.4059936mm" height="1.397mm" shape="rect" />
<silkscreenpath route={[{"x":-1.4784070000000042,"y":-0.6477000000000004},{"x":-1.4784070000000042,"y":-1.2999974000000094},{"x":1.4999970000000076,"y":-1.2999974000000094},{"x":1.4999970000000076,"y":1.2999973999999952},{"x":-1.4784070000000042,"y":1.2999973999999952},{"x":-1.4784070000000042,"y":0.6477000000000004}]} />
<silkscreenpath route={[{"x":-1.4784070000000042,"y":0.6477000000000004},{"x":-1.4784070000000042,"y":-0.6477000000000004}]} />
      </footprint>}
      cadModel={{
        objUrl: "https://modelcdn.tscircuit.com/easyeda_models/download?uuid=ad9a0033bb724b3cb918db51ef2b8c4f&pn=C124467",
        rotationOffset: { x: 0, y: 0, z: 0 },
        positionOffset: { x: 0, y: 0, z: 0 },
      }}
      {...props}
    />
  )
}