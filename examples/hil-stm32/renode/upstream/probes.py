# Probes for the three Renode 1.16.1 gaps (monitor-level IronPython, ASCII only).
def mc_pin(t, n):
    print "[pin] %s ch%d IsSet=%s" % (t.GetType().Name, int(n), t.Connections[int(n)].IsSet)

def mc_irq(u):
    print "[irq] %s IRQ.IsSet=%s" % (u.GetType().Name, u.IRQ.IsSet)
