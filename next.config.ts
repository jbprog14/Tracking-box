import type {NextConfig} from "next";

const nextConfig: NextConfig = {
  // output: "export", // Removed to allow dynamic routes
  trailingSlash: true,
  images: {
    unoptimized: true,
  },
  // Ensure we're not using static export for Cloudflare Pages
  // which needs to handle dynamic routes
};

export default nextConfig;
