import QRDeviceClient from './QRDeviceClient';

// This function is required for static export but will allow dynamic routes at runtime
export async function generateStaticParams() {
  // Return empty array to allow any deviceId to be handled dynamically
  return [];
}

interface PageProps {
  params: { deviceId: string };
}

export default function QRDevicePage({ params }: PageProps) {
  return <QRDeviceClient deviceId={params.deviceId} />;
}
