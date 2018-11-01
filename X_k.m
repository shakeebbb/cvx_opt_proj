function X_k = X_k(X,n,padding)
X_k = circshift(X,n);
X_k(1:n) = padding;
end