"use client";

import React, { useState, useEffect } from "react";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
} from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { MapPin, User, Save, X } from "lucide-react";

interface EditInfoModalProps {
  isOpen: boolean;
  onClose: () => void;
  boxId: string | null;
  currentAddress: string; // human-readable label
  currentOwner: string;
  onSave: (boxId: string, coords: string, owner: string, label: string) => void;
}

export default function EditInfoModal({
  isOpen,
  onClose,
  boxId,
  currentAddress,
  currentOwner,
  onSave,
}: EditInfoModalProps) {
  const [address, setAddress] = useState("");
  const [owner, setOwner] = useState("");
  const [isSaving, setIsSaving] = useState(false);
  const [geoError, setGeoError] = useState("");

  // Reset fields when modal opens
  useEffect(() => {
    if (isOpen) {
      setAddress(currentAddress || "");
      setOwner(currentOwner || "");
    }
  }, [isOpen, currentAddress, currentOwner]);

  // --- Geocoding helper (tries multiple services) ---
  const geocodeAddress = async (
    query: string
  ): Promise<{ lat: string; lon: string } | null> => {
    // 1) Nominatim
    try {
      const nominatimUrl = `https://nominatim.openstreetmap.org/search?format=jsonv2&limit=1&q=${encodeURIComponent(
        query
      )}`;
      const res = await fetch(nominatimUrl, {
        headers: {
          // The browser will strip User-Agent; this is fine. We keep it for SSR environments.
          "User-Agent": "tracking-box-dashboard",
          "Accept-Language": "en",
        },
      });
      const data = await res.json();
      if (data && data.length > 0) {
        return { lat: data[0].lat, lon: data[0].lon };
      }
    } catch (err) {
      console.warn("Nominatim geocode failed", err);
    }

    // 2) Fallback – geocode.maps.co (public wrapper around OSM, CORS-enabled)
    try {
      const fallbackUrl = `https://geocode.maps.co/search?q=${encodeURIComponent(
        query
      )}`;
      const res = await fetch(fallbackUrl);
      const data = await res.json();
      if (data && data.length > 0) {
        return { lat: data[0].lat, lon: data[0].lon };
      }
    } catch (err) {
      console.warn("Fallback geocode failed", err);
    }

    return null;
  };

  const handleSave = async () => {
    if (!boxId || !address.trim() || !owner.trim()) return;

    setIsSaving(true);
    setGeoError("");

    try {
      const coords = await geocodeAddress(address.trim());

      if (!coords) {
        setGeoError(
          "We couldn't locate that address. Please refine it or enter coordinates manually."
        );
        return;
      }

      const finalAddress = `${coords.lat}, ${coords.lon}`;
      onSave(boxId, finalAddress, owner.trim(), address.trim());
      onClose();
    } catch (error) {
      console.error("Geocoding error:", error);
      setGeoError("Unexpected error while locating address.");
    } finally {
      setIsSaving(false);
    }
  };

  const handleCancel = () => {
    setAddress(currentAddress || "");
    setOwner(currentOwner || "");
    onClose();
  };

  return (
    <Dialog open={isOpen} onOpenChange={onClose}>
      <DialogContent className="sm:max-w-md">
        <DialogHeader>
          <DialogTitle className="text-xl font-bold text-gray-900 flex items-center gap-2">
            <div className="p-2 bg-blue-600 rounded-md border-2 border-blue-700">
              <User className="h-5 w-5 text-white" />
            </div>
            Edit Tracking Box Info
          </DialogTitle>
        </DialogHeader>

        <div className="space-y-6 py-4">
          {boxId && (
            <div className="text-center">
              <div className="inline-flex items-center px-3 py-1 rounded-md text-sm font-medium bg-gray-200 text-gray-800 border border-gray-400">
                Box ID: {boxId.toUpperCase()}
              </div>
            </div>
          )}

          {/* Owner Field */}
          <div className="space-y-1">
            <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
              <User className="h-4 w-4 text-purple-600" />
              Owner
            </label>
            <input
              type="text"
              value={owner}
              onChange={(e) => setOwner(e.target.value)}
              placeholder="Enter the owner's name..."
              className="w-full p-3 border-2 border-gray-300 rounded-md text-sm"
              maxLength={50}
            />
            <div className="text-xs text-gray-400">{owner.length}/50</div>
          </div>

          {/* Address Field */}
          <div className="space-y-1">
            <label className="flex items-center gap-2 text-sm font-bold text-gray-700">
              <MapPin className="h-4 w-4 text-blue-600" />
              Address
            </label>
            <div className="relative">
              <textarea
                value={address}
                onChange={(e) => setAddress(e.target.value)}
                placeholder="Enter the shipping address..."
                className="w-full p-3 border-2 border-gray-300 rounded-md resize-none h-20 text-sm"
                maxLength={200}
              />
              <div className="absolute bottom-2 right-2 text-xs text-gray-400">
                {address.length}/200
              </div>
            </div>
            {geoError && (
              <div className="bg-red-50 border border-red-200 rounded-lg p-3">
                <p className="text-red-700 text-sm">{geoError}</p>
              </div>
            )}
          </div>

          {/* Action Buttons */}
          <div className="flex gap-3 pt-4">
            <Button
              onClick={handleCancel}
              variant="outline"
              className="flex-1 flex items-center gap-2 border-2 border-gray-400 rounded-md hover:bg-gray-100 hover:border-gray-500 text-gray-700"
            >
              <X className="h-4 w-4" />
              Cancel
            </Button>
            <Button
              onClick={handleSave}
              disabled={!address.trim() || !owner.trim() || isSaving}
              className="flex-1 flex items-center gap-2 bg-blue-600 hover:bg-blue-700 text-white border-2 border-blue-700 rounded-md disabled:opacity-50 disabled:cursor-not-allowed disabled:bg-gray-400 disabled:border-gray-500"
            >
              {isSaving ? (
                "Saving..."
              ) : (
                <>
                  <Save className="h-4 w-4" />
                  Save Changes
                </>
              )}
            </Button>
          </div>

          {/* Warning Text */}
          <p className="text-xs text-gray-500 text-center bg-gray-100 p-2 rounded-md border border-gray-300">
            Note: Changes will be applied immediately and synced to the
            database.
          </p>
        </div>
      </DialogContent>
    </Dialog>
  );
}
