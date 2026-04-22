#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/energy-module.h"

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace ns3;

/* =========================================================
   Adaptive UDP Application
   ========================================================= */
class AdaptiveUdpApp : public Application
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("AdaptiveUdpApp")
                            .SetParent<Application>()
                            .SetGroupName("Tutorial")
                            .AddConstructor<AdaptiveUdpApp>();
    return tid;
  }

  AdaptiveUdpApp()
      : m_socket(0),
        m_running(false),
        m_peerAddress(Ipv4Address("0.0.0.0")),
        m_peerPort(0),
        m_packetSize(128),
        m_maxPackets(100),
        m_sent(0),
        m_interval(MilliSeconds(20)),
        m_priority(0)
  {
  }

  void Setup(Ipv4Address peerAddress,
             uint16_t peerPort,
             uint32_t packetSize,
             uint32_t maxPackets,
             Time interval,
             uint8_t priority)
  {
    m_peerAddress = peerAddress;
    m_peerPort = peerPort;
    m_packetSize = packetSize;
    m_maxPackets = maxPackets;
    m_interval = interval;
    m_priority = priority;
  }

  void SetInterval(Time interval)
  {
    m_interval = interval;
  }

  Time GetInterval() const
  {
    return m_interval;
  }

  uint8_t GetPriority() const
  {
    return m_priority;
  }

private:
  virtual void StartApplication(void)
  {
    m_running = true;
    m_sent = 0;

    if (!m_socket)
    {
      m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
      InetSocketAddress remote = InetSocketAddress(m_peerAddress, m_peerPort);
      m_socket->Connect(remote);
    }

    SendPacket();
  }

  virtual void StopApplication(void)
  {
    m_running = false;

    if (m_sendEvent.IsPending())
    {
      Simulator::Cancel(m_sendEvent);
    }

    if (m_socket)
    {
      m_socket->Close();
    }
  }

  void SendPacket(void)
  {
    if (!m_running || m_sent >= m_maxPackets)
    {
      return;
    }

    Ptr<Packet> packet = Create<Packet>(m_packetSize);
    m_socket->Send(packet);
    m_sent++;

    ScheduleTx();
  }

  void ScheduleTx(void)
  {
    if (m_running && m_sent < m_maxPackets)
    {
      m_sendEvent = Simulator::Schedule(m_interval, &AdaptiveUdpApp::SendPacket, this);
    }
  }

private:
  Ptr<Socket> m_socket;
  bool m_running;
  EventId m_sendEvent;

  Ipv4Address m_peerAddress;
  uint16_t m_peerPort;

  uint32_t m_packetSize;
  uint32_t m_maxPackets;
  uint32_t m_sent;
  Time m_interval;
  uint8_t m_priority; // 1 = high priority, 0 = normal priority
};

/* =========================================================
   Global controller variables
   ========================================================= */
static Ptr<FlowMonitor> g_monitor;
static std::vector< Ptr<ns3::energy::EnergySource> > g_sources;
static std::vector<Ptr<AdaptiveUdpApp>> g_apps;

static double g_initialEnergy = 25.0;
static double g_lastPredictedScore = 0.0;
static uint32_t g_numNodes = 10;
static double g_simTime = 20.0;

/* =========================================================
   Utility functions
   ========================================================= */
double Clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

/*
 * DL-assistance placeholder.
 * Replace this with real model inference later if needed.
 */
double PredictCongestionScore(double avgDelay,
                              double avgJitter,
                              double lossRatio,
                              double energyStress)
{
  double delayNorm = Clamp(avgDelay / 0.20, 0.0, 1.0);
  double jitterNorm = Clamp(avgJitter / 0.10, 0.0, 1.0);
  double lossNorm = Clamp(lossRatio, 0.0, 1.0);
  double energyNorm = Clamp(energyStress, 0.0, 1.0);

  double instantScore =
      0.35 * delayNorm +
      0.20 * jitterNorm +
      0.30 * lossNorm +
      0.15 * energyNorm;

  double predictedScore = 0.7 * instantScore + 0.3 * g_lastPredictedScore;
  g_lastPredictedScore = predictedScore;

  return Clamp(predictedScore, 0.0, 1.0);
}

/* =========================================================
   Periodic congestion controller
   ========================================================= */
void CongestionController()
{
  g_monitor->CheckForLostPackets();

  std::map<FlowId, FlowMonitor::FlowStats> stats = g_monitor->GetFlowStats();

  uint32_t txPackets = 0;
  uint32_t rxPackets = 0;
  uint32_t lostPackets = 0;

  double totalDelay = 0.0;
  double totalJitter = 0.0;

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator it = stats.begin();
       it != stats.end(); ++it)
  {
    txPackets += it->second.txPackets;
    rxPackets += it->second.rxPackets;
    lostPackets += it->second.lostPackets;
    totalDelay += it->second.delaySum.GetSeconds();
    totalJitter += it->second.jitterSum.GetSeconds();
  }

  double avgDelay = 0.0;
  double avgJitter = 0.0;
  double lossRatio = 0.0;

  if (rxPackets > 0)
  {
    avgDelay = totalDelay / rxPackets;
    avgJitter = totalJitter / rxPackets;
  }

  if (txPackets > 0)
  {
    lossRatio = static_cast<double>(lostPackets) / static_cast<double>(txPackets);
  }

  double totalRemaining = 0.0;
  for (auto const &src : g_sources)
  {
    totalRemaining += src->GetRemainingEnergy();
  }

  double avgRemaining = totalRemaining / g_numNodes;
  double energyStress = 1.0 - (avgRemaining / g_initialEnergy);

  double congestionScore = PredictCongestionScore(avgDelay, avgJitter, lossRatio, energyStress);

  for (std::vector<Ptr<AdaptiveUdpApp>>::iterator it = g_apps.begin(); it != g_apps.end(); ++it)
  {
    Ptr<AdaptiveUdpApp> app = *it;
    double currentMs = app->GetInterval().GetMilliSeconds();
    double newMs = currentMs;

    if (congestionScore > 0.70)
    {
      if (app->GetPriority() == 1)
      {
        newMs = currentMs * 1.15;
      }
      else
      {
        newMs = currentMs * 1.35;
      }
    }
    else if (congestionScore > 0.50)
    {
      if (app->GetPriority() == 1)
      {
        newMs = currentMs * 1.05;
      }
      else
      {
        newMs = currentMs * 1.20;
      }
    }
    else if (congestionScore < 0.25)
    {
      newMs = currentMs * 0.90;
    }

    newMs = Clamp(newMs, 10.0, 200.0);
    app->SetInterval(MilliSeconds(newMs));
  }

  std::cout << std::fixed << std::setprecision(4)
            << "[Controller @ " << Simulator::Now().GetSeconds() << "s] "
            << "PredictedCongestionScore=" << congestionScore
            << " AvgDelay=" << avgDelay
            << " AvgJitter=" << avgJitter
            << " LossRatio=" << lossRatio
            << " EnergyStress=" << energyStress
            << std::endl;

  if (Simulator::Now().GetSeconds() + 1.0 < g_simTime)
  {
    Simulator::Schedule(Seconds(1.0), &CongestionController);
  }
}

/* =========================================================
   Main function
   ========================================================= */
int main(int argc, char *argv[])
{
  uint32_t runNumber = 1;
  uint32_t numNodes = 20;
  double simTime = 30.0;
  uint32_t packetSize = 128;
  uint32_t totalPackets = 1000;
  bool enableAnim = true;
  std::string animFile = "dl-assisted-wsn-congestion.xml";

  CommandLine cmd;
  cmd.AddValue("run", "Run number", runNumber);
  cmd.AddValue("numNodes", "Number of nodes", numNodes);
  cmd.AddValue("simTime", "Simulation time", simTime);
  cmd.AddValue("enableAnim", "Enable NetAnim XML generation", enableAnim);
  cmd.AddValue("animFile", "NetAnim XML filename", animFile);
  cmd.Parse(argc, argv);

  g_numNodes = numNodes;
  g_simTime = simTime;

  RngSeedManager::SetSeed(1);
  RngSeedManager::SetRun(runNumber);

  NodeContainer nodes;
  nodes.Create(numNodes);

  /* ---------------- WIFI ---------------- */
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211b);

  YansWifiChannelHelper channel;
  channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  channel.AddPropagationLoss("ns3::FriisPropagationLossModel");

  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());

  WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  /* ---------------- MOBILITY ---------------- */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

  uint32_t gridWidth = std::ceil(std::sqrt(static_cast<double>(numNodes)));
  double delta = 20.0;

  for (uint32_t i = 0; i < numNodes; ++i)
  {
    uint32_t row = i / gridWidth;
    uint32_t col = i % gridWidth;
    positionAlloc->Add(Vector(col * delta, row * delta, 0.0));
  }

  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  /* ---------------- ROUTING ---------------- */
  DsdvHelper dsdv;
  InternetStackHelper internet;
  internet.SetRoutingHelper(dsdv);
  internet.Install(nodes);

  /* ---------------- IP ADDRESS ---------------- */
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  /* ---------------- ENERGY MODEL ---------------- */
  BasicEnergySourceHelper energySource;
  energySource.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(g_initialEnergy));

  ns3::energy::EnergySourceContainer sourceContainer = energySource.Install(nodes);
  g_sources.clear();
  for (uint32_t i = 0; i < sourceContainer.GetN(); ++i)
  {
    g_sources.push_back(sourceContainer.Get(i));
  }

  WifiRadioEnergyModelHelper radioEnergy;
  radioEnergy.Install(devices, sourceContainer);

  /* ---------------- TRAFFIC FLOWS ---------------- */
  Ptr<UniformRandomVariable> randNode = CreateObject<UniformRandomVariable>();

  uint32_t numFlows = numNodes / 2;
  if (numFlows == 0)
  {
    numFlows = 1;
  }

  uint32_t packetsPerFlow = totalPackets / numFlows;
  uint16_t basePort = 9000;

  for (uint32_t i = 0; i < numFlows; ++i)
  {
    uint32_t src = randNode->GetInteger(0, numNodes - 1);
    uint32_t dst = randNode->GetInteger(0, numNodes - 1);

    while (src == dst)
    {
      dst = randNode->GetInteger(0, numNodes - 1);
    }

    uint16_t port = basePort + i;

    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));

    ApplicationContainer sinkApp = sinkHelper.Install(nodes.Get(dst));
    sinkApp.Start(Seconds(0.5));
    sinkApp.Stop(Seconds(simTime));

    Ptr<AdaptiveUdpApp> app = CreateObject<AdaptiveUdpApp>();

    uint8_t priority = (i % 3 == 0) ? 1 : 0;
    Time initInterval = MilliSeconds(15 + (i % 4) * 5);

    app->Setup(interfaces.GetAddress(dst),
               port,
               packetSize,
               packetsPerFlow,
               initInterval,
               priority);

    nodes.Get(src)->AddApplication(app);
    app->SetStartTime(Seconds(1.0 + 0.2 * i));
    app->SetStopTime(Seconds(simTime));

    g_apps.push_back(app);
  }

  /* ---------------- FLOW MONITOR ---------------- */
  FlowMonitorHelper flowmon;
  g_monitor = flowmon.InstallAll();

  /* ---------------- NETANIM ---------------- */
  std::unique_ptr<AnimationInterface> anim;
  if (enableAnim)
  {
    anim = std::make_unique<AnimationInterface>(animFile);
    anim->SetMaxPktsPerTraceFile(500000);
    anim->EnablePacketMetadata(true);

    for (uint32_t i = 0; i < numNodes; ++i)
    {
      std::ostringstream os;
      os << "N" << i;
      anim->UpdateNodeDescription(nodes.Get(i), os.str());

      if (i == 0)
      {
        anim->UpdateNodeColor(nodes.Get(i), 255, 0, 0);
      }
      else
      {
        anim->UpdateNodeColor(nodes.Get(i), 0, 0, 255);
      }

      Vector pos = nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
      anim->SetConstantPosition(nodes.Get(i), pos.x, pos.y);
    }
  }

  /* ---------------- CONTROLLER START ---------------- */
  Simulator::Schedule(Seconds(2.0), &CongestionController);

  /* ---------------- RUN ---------------- */
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  g_monitor->CheckForLostPackets();

  std::map<FlowId, FlowMonitor::FlowStats> stats = g_monitor->GetFlowStats();

  double totalDelay = 0.0;
  double totalJitter = 0.0;
  uint32_t txPackets = 0;
  uint32_t rxPackets = 0;
  uint32_t lostPackets = 0;
  double throughput = 0.0;

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator it = stats.begin();
       it != stats.end(); ++it)
  {
    txPackets += it->second.txPackets;
    rxPackets += it->second.rxPackets;
    lostPackets += it->second.lostPackets;

    totalDelay += it->second.delaySum.GetSeconds();
    totalJitter += it->second.jitterSum.GetSeconds();

    throughput += (it->second.rxBytes * 8.0) / simTime / 1024.0;
  }

  double avgDelay = 0.0;
  double avgJitter = 0.0;
  if (rxPackets > 0)
  {
    avgDelay = totalDelay / rxPackets;
    avgJitter = totalJitter / rxPackets;
  }

  double pdr = 0.0;
  if (txPackets > 0)
  {
    pdr = (static_cast<double>(rxPackets) / static_cast<double>(txPackets)) * 100.0;
  }

  double communicationOverhead = lostPackets;
  double computationalOverhead = avgDelay * rxPackets;

  double totalEnergyConsumed = 0.0;
  for (auto const &src : g_sources)
  {
    totalEnergyConsumed += (g_initialEnergy - src->GetRemainingEnergy());
  }

  double avgEnergyConsumed = totalEnergyConsumed / numNodes;

  std::cout << "\n----- Deep Learning-Assisted Congestion Control Results -----\n";
  std::cout << "Packets Sent = " << txPackets << std::endl;
  std::cout << "Packets Received = " << rxPackets << std::endl;
  std::cout << "Packets Lost = " << lostPackets << std::endl;
  std::cout << "Packet Delivery Ratio (PDR) = " << pdr << " %\n";
  std::cout << "End-to-End Delay = " << avgDelay << " sec\n";
  std::cout << "Average Jitter = " << avgJitter << " sec\n";
  std::cout << "Throughput = " << throughput << " Kbps\n";
  std::cout << "Communication Overhead = " << communicationOverhead << " pkts\n";
  std::cout << "Computational Overhead = " << computationalOverhead << " sec\n";
  std::cout << "Average Energy Consumption = " << avgEnergyConsumed << " Joules\n";
  std::cout << "Final Predicted Congestion Score = " << g_lastPredictedScore << std::endl;

  g_monitor->SerializeToXmlFile("flowmon.xml", true, true);

  std::ofstream file;
  file.open("results.csv", std::ios::app);

  file << numNodes << ","
       << pdr << ","
       << avgDelay << ","
       << avgJitter << ","
       << throughput << ","
       << avgEnergyConsumed << std::endl;

  file.close();

  Simulator::Destroy();
  return 0;
}
