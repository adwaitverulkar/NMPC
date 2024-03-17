function sigma = sigmoid(x)
    
    gain = 1000;
    sigma = 1./(1+exp(-gain*x));

end