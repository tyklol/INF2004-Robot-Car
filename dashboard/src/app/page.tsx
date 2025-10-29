"use client";

import Image from "next/image";
import { useEffect, useState, useMemo } from "react";

type MqttPayload = { topic: string; data: any; time: string };

function useMqtt(brokerUrl: string, topic: string) {
  const [messages, setMessages] = useState<MqttPayload[]>([]);

  useEffect(() => {
    let client: any | null = null;
    let mounted = true;

    (async () => {
      try {
        const mqttModule = await import("mqtt");
        const mqtt = mqttModule.default ?? mqttModule;

        client = mqtt.connect(brokerUrl, {
          clientId: "dash-" + Math.random().toString(16).slice(2, 10),
          reconnectPeriod: 2000,
        });

        client.on("connect", () => {
          if (!mounted) return;
          client.subscribe(topic, (err: Error) => {
            if (err) console.error("MQTT subscribe error:", err);
          });
        });

        client.on("message", (t: string, payload: Buffer | string) => {
          if (!mounted) return;
          try {
            const str = payload instanceof Buffer ? payload.toString() : String(payload);
            const obj = JSON.parse(str);
            setMessages((m) => [...m.slice(-999), { topic: t, data: obj, time: new Date().toISOString() }]);
          } catch (err) {
            const str = payload instanceof Buffer ? payload.toString() : String(payload);
            setMessages((m) => [...m.slice(-999), { topic: t, data: str, time: new Date().toISOString() }]);
          }
        });
      } catch (err) {
        console.error("Failed to initialize mqtt client:", err);
      }
    })();

    return () => {
      mounted = false;
      try {
        if (client) client.end(true);
      } catch (e) {
        // ignore
      }
    };
  }, [brokerUrl, topic]);

  return messages;
}

export default function Home() {
  // const broker = "ws://10.114.145.205:8080";
  const broker = "ws://localhost:8080";
  const topic = "#";
  const messages = useMqtt(broker, topic);

  // group messages by topic
  const topicEntries = useMemo(() => {
    const map: Record<string, MqttPayload[]> = {};
    for (const m of messages) {
      if (!map[m.topic]) map[m.topic] = [];
      map[m.topic].push(m);
    }
    return Object.entries(map); // [ [topic, msgs], ... ]
  }, [messages]);

  return (
    <div className="min-h-screen bg-gray-50 dark:bg-neutral-900 text-neutral-900 dark:text-neutral-100">
      <header className="max-w-7xl mx-auto px-6 py-6 flex items-center justify-between">
        <div className="flex items-center gap-4">
          <Image src="/next.svg" alt="logo" width={48} height={12} />
          <div>
            <h1 className="text-xl font-semibold">MQTT Dashboard — All Topics</h1>
            <p className="text-sm text-neutral-500 dark:text-neutral-400">Subscribed to "{topic}" on {broker}</p>
          </div>
        </div>

        <div className="flex items-center gap-3">
          <a
            className="rounded-md bg-black text-white px-3 py-2 text-sm"
            href="#"
            onClick={(e) => e.preventDefault()}
          >
            Live
          </a>
          <a
            className="text-sm text-neutral-500 dark:text-neutral-400 underline"
            href="https://mqtt.org/"
            target="_blank"
            rel="noreferrer"
          >
            MQTT docs
          </a>
        </div>
      </header>

      <main className="max-w-7xl mx-auto px-6 pb-16">
        {/* Controls */}
        <section className="mb-6">
          <div className="flex flex-wrap items-center gap-3">
            <div className="text-sm text-neutral-500 dark:text-neutral-400">Showing topics:</div>
            <div className="px-3 py-1 rounded bg-white dark:bg-neutral-800 border text-sm">{topicEntries.length} topics</div>
            <div className="ml-auto text-sm text-neutral-500 dark:text-neutral-400">Newest topics first</div>
          </div>
        </section>

        {/* Grid of topic cards - responsive */}
        <section className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-6">
          {topicEntries.length === 0 && (
            <div className="col-span-full rounded-md border border-dashed border-neutral-200 dark:border-neutral-800 p-8 text-center text-neutral-500">
              No messages yet.
            </div>
          )}

          {topicEntries.map(([t, msgs]) => (
            <article key={t} className="rounded-lg bg-white dark:bg-neutral-800 shadow-sm border dark:border-neutral-700 overflow-hidden flex flex-col">
              <div className="px-4 py-3 border-b dark:border-neutral-700 flex items-center justify-between">
                <div className="text-sm font-medium truncate">{t}</div>
                <div className="text-xs text-neutral-500 dark:text-neutral-400">{msgs.length} msgs</div>
              </div>

              <div className="p-3 flex-1 overflow-auto text-xs space-y-2">
                {msgs.slice().reverse().slice(0, 8).map((m, i) => (
                  <div key={i} className="rounded bg-neutral-50 dark:bg-neutral-900 p-2">
                    <div className="text-[10px] text-neutral-400">{new Date(m.time).toLocaleTimeString()}</div>
                    <pre className="whitespace-pre-wrap max-h-28 overflow-auto text-[12px] text-neutral-700 dark:text-neutral-200">
                      {typeof m.data === "string" ? m.data : JSON.stringify(m.data, null, 2)}
                    </pre>
                  </div>
                ))}
              </div>

              <div className="px-3 py-2 border-t dark:border-neutral-700 flex items-center gap-2">
                <button
                  className="text-xs px-2 py-1 rounded bg-black text-white"
                  onClick={() => {
                    // scroll to top of this card (quick local action)
                    window.requestAnimationFrame(() => {
                      /* noop — kept small for potential enhancement */
                    });
                  }}
                >
                  Latest
                </button>
                <a
                  className="ml-auto text-xs text-neutral-500 dark:text-neutral-400 underline"
                  href="#"
                  onClick={(e) => {
                    e.preventDefault();
                    // quick raw view in new window
                    const payloads = msgs.map((m) => `${m.time} — ${typeof m.data === "string" ? m.data : JSON.stringify(m.data)}`).join("\n\n");
                    const w = window.open("", "_blank");
                    if (w) {
                      w.document.write(`<pre>${payloads.replaceAll("&", "&amp;").replaceAll("<", "&lt;")}</pre>`);
                      w.document.title = `Topic ${t}`;
                    }
                  }}
                >
                  Raw
                </a>
              </div>
            </article>
          ))}
        </section>
      </main>
    </div>
  );
}