import type {NextConfig} from "next";

const nextConfig: NextConfig = {
  // Removed output: "export" to allow SSR on Cloudflare Pages
  trailingSlash: true,
  images: {
    unoptimized: true,
  },
};

export default nextConfig;
