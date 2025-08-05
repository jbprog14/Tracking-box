"use client";

import React from "react";
import QRCode from "react-qr-code";

const QRLinkPage = () => {
  // Default to box_001, but this could be made dynamic based on URL params or props
  const deviceId = "box_001";
  const url = `https://tracking-box.vercel.app/qr/${deviceId}/`;

  return (
    <main className="flex min-h-screen flex-col items-center justify-center bg-gray-100 p-24">
      <div className="w-full max-w-md bg-white rounded-lg shadow-md border border-gray-300 p-8 text-center">
        <h1 className="text-3xl font-bold text-gray-900 mb-6">Scan QR Code</h1>
        <div
          className="flex justify-center"
          style={{ background: "white", padding: "16px" }}
        >
          <QRCode value={url} size={256} />
        </div>
        <p className="mt-6 text-sm text-gray-600">
          Scan this QR code to visit:{" "}
          <a
            href={url}
            target="_blank"
            rel="noopener noreferrer"
            className="text-blue-600 hover:underline"
          >
            {url}
          </a>
        </p>
      </div>
    </main>
  );
};

export default QRLinkPage;
