import React, { useState } from 'react';
import { AppProviders, useUIContext } from './context';
import { OfflineBanner } from './components/ui/OfflineBanner';
import { TopBar } from './components/TopBar';
import { TabNavigation } from './components/TabNavigation';
import { OverviewTab } from './components/tabs/OverviewTab';
import { MissionTab } from './components/tabs/MissionTab';
import { CognitionTab } from './components/tabs/CognitionTab';
import { PerceptionTab } from './components/tabs/PerceptionTab';
import { CommunicationTab } from './components/tabs/CommunicationTab';
import { BlackboardTab } from './components/tabs/BlackboardTab';
import { TestingTab } from './components/tabs/TestingTab';
import { DebugTab } from './components/tabs/DebugTab';
import { DebuggingDashboard } from './components/debugging';
import { ConfigTab } from './components/tabs/ConfigTab';
import { SectionErrorBoundary } from './components/debugging/UnifiedDebugging/SectionErrorBoundary';

/**
 * Main App Component
 *
 * Clean, modular, context-aware interface for URC 2026 rover control system.
 * Features:
 * - Tab-based navigation
 * - Context-aware UI that adapts to system state
 * - Information-dense displays
 * - Progressive disclosure
 */
function AppContent() {
  const [activeTab, setActiveTab] = useState('overview');
  const { isOnline } = useUIContext();

  const renderTabContent = () => {
    switch (activeTab) {
      case 'overview':
        return (
          <SectionErrorBoundary>
            <OverviewTab />
          </SectionErrorBoundary>
        );
      case 'mission':
        return (
          <SectionErrorBoundary>
            <MissionTab />
          </SectionErrorBoundary>
        );
      case 'network':
        return (
          <SectionErrorBoundary>
            <CognitionTab />
          </SectionErrorBoundary>
        );
      case 'perception':
        return (
          <SectionErrorBoundary>
            <PerceptionTab />
          </SectionErrorBoundary>
        );
      case 'communication':
        return (
          <SectionErrorBoundary>
            <CommunicationTab />
          </SectionErrorBoundary>
        );
      case 'testing':
        return (
          <SectionErrorBoundary>
            <TestingTab />
          </SectionErrorBoundary>
        );
      case 'debug':
        return (
          <SectionErrorBoundary>
            <DebuggingDashboard />
          </SectionErrorBoundary>
        );
      case 'blackboard':
        return (
          <SectionErrorBoundary>
            <BlackboardTab />
          </SectionErrorBoundary>
        );
      case 'config':
        return (
          <SectionErrorBoundary>
            <ConfigTab />
          </SectionErrorBoundary>
        );
      default:
        return (
          <SectionErrorBoundary>
            <OverviewTab />
          </SectionErrorBoundary>
        );
    }
  };

  return (
    <div className="min-h-screen w-screen flex flex-col bg-zinc-950 text-zinc-100">
      {/* Top bar - always visible */}
      <TopBar />

      {/* Offline banner */}
      <OfflineBanner
        isOnline={isOnline}
        onRetry={() => window.location.reload()}
        lastSeen={null} // TODO: Add last connection timestamp
      />

      {/* Tab navigation */}
      <TabNavigation activeTab={activeTab} setActiveTab={setActiveTab} />

      {/* Tab content - scrollable */}
      <div className="flex-1 overflow-y-auto min-h-0">
        {renderTabContent()}
      </div>
    </div>
  );
}

function App() {
  return (
    <AppProviders>
      <AppContent />
    </AppProviders>
  );
}

export default App;
