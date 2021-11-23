function v = INTERNAL()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 120);
  end
  v = vInitialized;
end
