import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "MQTT Dashboard",
  description: "Live MQTT topics dashboard",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className="antialiased bg-gray-50 dark:bg-neutral-900">
        {children}
      </body>
    </html>
  );
}
