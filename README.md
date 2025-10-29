# INF2004-Robot-Car
<!-- FOR MQTT -->
NOTE TO CHANGE CMAKE ENVIRONMENT VARIABLE TO YOUR OWN FOR:
WIFI_SSID <your hotspot or wifi>
WIFI_PASSWORD <your password>
MQTT_SERVER <your IPV4 Address>

Download MQTT from https://mosquitto.org/download/
(I selected mosquitto-2.0.22-install-windows-x64.exe under Windows)
Go find mosquitto.conf (mine was at C:\Program Files\mosquitto\mosquitto.conf) and add the following line to the file:
listener 1883
allow_anonymous true
listener 8080
protocol websockets

Restart the mqtt service with this command in an Administrator Command Prompt:
net stop mosquitto && net start mosquitto

Open a command prompt and type in the following command to check if MQTT accepts connections from any IP:
netstat -ano | findstr :1883
Output should be something like this if MQTT accepts connections from any IP:
TCP    0.0.0.0:1883           0.0.0.0:0              LISTENING       33608
TCP    [::]:1883              [::]:0                 LISTENING       33608

If firewall is fucking with the MQTT, type in the following command:
netsh advfirewall firewall add rule name="Mosquitto MQTT" dir=in action=allow protocol=TCP localport=1883

Open a command prompt and type in the following command to see any message being published to MQTT:
mosquitto_sub -h localhost -t "#" -v

Command to publish simple message to different topics:
mosquitto_pub -h localhost -t "/led" -m "On"
mosquitto_pub -h localhost -t "/led" -m "Off"
mosquitto_pub -h localhost -t "/print" -m "hello from PC"
<!-- FOR Dashboard -->
Run the following commands:

cd dashboard
npm install
npm run dev

Go to [localhost:3000](http://localhost:3000)

This is a [Next.js](https://nextjs.org) project bootstrapped with [`create-next-app`](https://nextjs.org/docs/app/api-reference/cli/create-next-app).

## Getting Started

First, run the development server:

```bash
npm run dev
# or
yarn dev
# or
pnpm dev
# or
bun dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

You can start editing the page by modifying `app/page.tsx`. The page auto-updates as you edit the file.

This project uses [`next/font`](https://nextjs.org/docs/app/building-your-application/optimizing/fonts) to automatically optimize and load [Geist](https://vercel.com/font), a new font family for Vercel.

## Learn More

To learn more about Next.js, take a look at the following resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js features and API.
- [Learn Next.js](https://nextjs.org/learn) - an interactive Next.js tutorial.

You can check out [the Next.js GitHub repository](https://github.com/vercel/next.js) - your feedback and contributions are welcome!

## Deploy on Vercel

The easiest way to deploy your Next.js app is to use the [Vercel Platform](https://vercel.com/new?utm_medium=default-template&filter=next.js&utm_source=create-next-app&utm_campaign=create-next-app-readme) from the creators of Next.js.

Check out our [Next.js deployment documentation](https://nextjs.org/docs/app/building-your-application/deploying) for more details.