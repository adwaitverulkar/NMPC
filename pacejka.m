function F = pacejka(Fz, B, C, D, E, slip)

    F = Fz .* D .* sin(C .* atan2(B .* slip - E .* (B .* slip), 1));
end